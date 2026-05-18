EVAL_DIR=$1
DATASET_DIR=$2

export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
python ./scripts/grasp_gen_ur/test.py --eval_dir=${EVAL_DIR} \
                                      --dataset_dir=${DATASET_DIR} \
                                      --stability_config='envs/tasks/grasp_test_force.yaml' \
                                      --seed=42
