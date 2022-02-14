import csv
import argparse
import os
import copy

job_prof_idx = 0
exec_t_list = []
job_prof_info = []
pid_list = []

# accumulate execution time by node
acc_runtime = {}

# exec info by pid
proc_exec_info = {}

curr_cpu = -1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', '-n', help='node name', required=True)
    parser.add_argument('--prof', '-p', help='.dat file', default='tmp.dat')
    parser.add_argument('--deli', '-d', help='csv file delimiter', default=',')
    parser.add_argument('--tmp', '-t', action='store_true', help='If you have tmp.txt, enable it', default=False)
    args = parser.parse_args()

    job_profiling_file_path = os.getcwd() + '/' + args.node + '.csv'
    if not os.path.exists(job_profiling_file_path):
        print("csv file not exists : %s" % (job_profiling_file_path))
        exit(1)

    if not args.tmp and not os.path.exists(args.prof):
        print("CPU Profiling file not exists : %s" % (args.prof))
        exit(1)

    if args.prof.split(".") == 1 or args.prof.split(".")[-1] != 'dat':
        print("CPU Profiling file should be dat file!")
        exit(1)

    if not args.tmp:
        report_cmd = 'trace-cmd report ' + args.prof + '| grep switch > tmp.txt'
        os.system(report_cmd)
        print("[System] trace-cmd for dat file Complete")
    elif not os.path.exists('tmp.txt'):
        print("You should have tmp.txt or disable --tmp option")
        exit(1)
    else:
        print("[System] You already have tmp.txt")

    ### Matching PID in csv and .txt file
    # Node name : name in csv
    # Process name : name in trace-cmd
    with open(job_profiling_file_path, 'r') as f:
        reader = csv.reader(f, delimiter=args.deli)
        for line in reader:
            # Add job's release / finish time info
            job_prof_info.append({"start_t" : float(line[0]), "end_t" : float(line[1]), "pid" : int(line[2])})

            # Add name_matching 
            pid = int(line[2])
            if pid not in pid_list:
                pid_list.append(pid)

    for pid in pid_list:
        if pid not in proc_exec_info:
            proc_exec_info[pid] = {'slice_start_t' : 0, 'isRun' : False}

        acc_runtime[pid] = 0
                
    print("[System] Name Matching between csv and trace-cmd result Complete")

    ### Parse trace-cmd result 

    with open('tmp.txt', 'r') as f:
        tracecmd_raw = f.readlines()
        line_num = len(tracecmd_raw)
        line_idx = 0
        prev_idx = 0

        for pid in acc_runtime:
            acc_runtime[pid] = 0

        while line_idx < line_num:
            running_pid = job_prof_info[job_prof_idx]["pid"]
            line = tracecmd_raw[line_idx]
            ts = float(line.split(': ')[0].split()[-1]) / 1000000000
            src_pid = int(line.split(' ==> ')[0].split(' [')[-2].split()[-1].split(':')[-1])
            src_proc = line.split(' ==> ')[0].split(' [')[-2].split()[-1].split(':')[0]
            dst_pid = int(line.split(' ==> ')[1].split(':')[-1].split()[0])
            dst_proc = line.split(' ==> ')[1].split(':')[0]
            cpu = int(line.split('[')[1].split(']')[0])

            ### timestamp is larger than job finish time
            if ts > job_prof_info[job_prof_idx]['end_t'] or line_idx == line_num - 1:
                # Add remain execution time if running and remain isRun
                for pid in proc_exec_info:
                    if proc_exec_info[pid]["isRun"]:
                        # did not preemption during whole time
                        if proc_exec_info[pid]["slice_start_t"] == 0:
                            acc_runtime[pid] += (job_prof_info[job_prof_idx]['end_t'] - job_prof_info[job_prof_idx]['start_t'])
                        else:
                            acc_runtime[pid] += (job_prof_info[job_prof_idx]['end_t'] - proc_exec_info[pid]["slice_start_t"])

                for pid in proc_exec_info:
                    proc_exec_info[pid]["slice_start_t"] = 0

                exec_t_list.append(copy.deepcopy(acc_runtime))
                
                for pid in acc_runtime:
                    acc_runtime[pid] = 0

                job_prof_idx += 1
                curr_cpu = -1

                if job_prof_idx >= len(job_prof_info) or line_idx == line_num - 1:
                    break

                # Resolve overlap issue
                if job_prof_info[job_prof_idx]['start_t'] < job_prof_info[job_prof_idx-1]['end_t']:
                    while True:
                        line_idx -= 1
                        tmp_line = tracecmd_raw[line_idx]
                        tmp_ts = float(tmp_line.split(': ')[0].split()[-1]) / 1000000000
                        if tmp_ts < job_prof_info[job_prof_idx]['start_t'] or line_idx == 0:
                            break

            elif ts > job_prof_info[job_prof_idx]['start_t']:
                # context switch into target node (start of slice)
                if dst_pid in pid_list:
                    proc_exec_info[dst_pid]["slice_start_t"] = ts
                    proc_exec_info[dst_pid]["isRun"] = True
                # context switch from target node (end of slice)
                if src_pid in pid_list:
                    # not start case
                    if proc_exec_info[src_pid]["slice_start_t"] != 0:
                        acc_runtime[src_pid] += (ts - proc_exec_info[src_pid]["slice_start_t"])
                        proc_exec_info[src_pid]["isRun"] = False
                    # start case
                    else:
                        acc_runtime[src_pid] += (ts - job_prof_info[job_prof_idx]['start_t'])

            line_idx += 1

    print("[System] Parse trace-cmd result file Complete")

    if not args.tmp:
        os.remove('tmp.txt')

    job_prof_idx = 2
    output_file_path = os.getcwd() + '/' + args.node + "_res.csv"

    with open(output_file_path, 'w') as f:
        writer = csv.writer(f, delimiter=',')
        first_row = ['start_t', 'end_t', 'pid', 'node_name', 'response_t', 'exec_t', 'preemptioned']
        writer.writerow(first_row)

        while job_prof_idx < len(exec_t_list):
            res_data = [job_prof_info[job_prof_idx]['start_t'], job_prof_info[job_prof_idx]['end_t'], job_prof_info[job_prof_idx]['pid'],
                        args.node]

            res_t = job_prof_info[job_prof_idx]['end_t'] - job_prof_info[job_prof_idx]['start_t']
            
            total_exec_t = 0
            for pid in pid_list:
                total_exec_t += exec_t_list[job_prof_idx][pid]

            res_data.append(res_t)
            res_data.append(total_exec_t)
            res_data.append(res_t - total_exec_t)
            
            writer.writerow(res_data)

            job_prof_idx += 1

    print("[System] Output Completely Generated!")
