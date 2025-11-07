#!/bin/bash
#SBATCH --job-name=gen_server
#SBATCH --ntasks=1
#SBATCH --nodes=1
#SBATCH --time=3-00:00
#SBATCH --gres=gpu:0
#SBATCH --cpus-per-task=1
#SBATCH --output=/mnt/lustre/work/geiger/bjaeger25/garage_2_cleanup/results/logs/gen_server_%a_%A.out
#SBATCH --error=/mnt/lustre/work/geiger/bjaeger25/garage_2_cleanup/results/logs/gen_server_%a_%A.err
#SBATCH --partition=2080-galvani

# print info about current job
echo "START TIME: $(date)"
start=`date +%s`

for i in $(seq 1 1); do
  python -u collect_dataset_slurm.py &
done
wait

end=`date +%s`
runtime=$((end-start))
echo "END TIME: $(date)"
echo "Runtime: ${runtime}"