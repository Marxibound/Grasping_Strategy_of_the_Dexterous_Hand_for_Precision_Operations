import os
import torch
import json
import numpy as np
def get_together():
    print("当前工作目录为"+os.getcwd())
    dirname="output/result/"
    target_file=os.path.join(dirname,"result.json")
    mdata_list=[]
    files_num=7
    OBJECT_MESH_DIR = "mesh_files"
    for i in range(files_num):
        file_name=os.path.join(dirname,f"result{i}.json")
        if os.path.exists(file_name):
            print(f"文件{file_name}存在")
        else:
            print(f"文件{file_name}不存在")
            return
        
        with open(file_name,"r",encoding='utf-8') as f:
            data=json.load(f)
        
        for mdata in data["metadata"]:
            mdata
            mdata_list.append(mdata)

    num_per_object={}
    for mdata in mdata_list:
        object_name=mdata["object_name"]
        num_per_object[object_name]=num_per_object.get(object_name,0)+1
    whole_grasp_dict={
        "info":{
            "dataset_source": "Manual Control (Data Glove) + PyBullet Simulation",
            "hand_type": "inspire_hand_left",
            "num_per_object": num_per_object,
            "total_valid_grasps": len(mdata_list),
            "object_dir": OBJECT_MESH_DIR
        },
        "metadata":mdata_list
    }
    with open(target_file,"w",encoding='utf-8') as f:
        json.dump(whole_grasp_dict,f,indent=4, ensure_ascii=False)


def to_torch():
    print("当前工作目录为"+os.getcwd())
    dirname="output/result/"
    target_file=os.path.join(dirname,"result.pt")
    mdata_list=[]
    files_num=7
    OBJECT_MESH_DIR = "mesh_files"
    for i in range(files_num):
        file_name=os.path.join(dirname,f"result{i}.json")
        if os.path.exists(file_name):
            print(f"文件{file_name}存在")
        else:
            print(f"文件{file_name}不存在")
            return
        
        with open(file_name,"r",encoding='utf-8') as f:
            data=json.load(f)
        
        for mdata in data["metadata"]:
            mdata["translations"]=torch.tensor(mdata["translations"],dtype=torch.float32)
            mdata["joint_positions"]=torch.tensor(mdata["joint_positions"],dtype=torch.float32)
            rotations=torch.tensor(np.array(mdata["rotations"]),dtype=torch.float32)
            mdata["rotations"]=rotations
            mdata_list.append(mdata)

    num_per_object={}
    for mdata in mdata_list:
        object_name=mdata["object_name"]
        num_per_object[object_name]=num_per_object.get(object_name,0)+1
    whole_grasp_dict={
        "info":{
            "dataset_source": "Manual Control (Data Glove) + PyBullet Simulation",
            "hand_type": "inspire_hand_left",
            "num_per_object": num_per_object,
            "total_valid_grasps": len(mdata_list),
            "object_dir": OBJECT_MESH_DIR
        },
        "metadata":mdata_list
    }
    torch.save(whole_grasp_dict,target_file)
            

        

if __name__=="__main__":
    #get_together()

    to_torch()
