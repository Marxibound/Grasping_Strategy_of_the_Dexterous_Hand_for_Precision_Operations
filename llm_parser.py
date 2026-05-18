# llm_parser.py
import openai

# 动作基元定义（供 LLM 参考）
ACTION_PRIMITIVES = """
你可以生成以下动作基元序列（JSON数组）：
1. {"action": "arm_move_joints", "joints": [j0,...,j5]}  // 绝对关节角（弧度）
2. {"action": "arm_move_joint", "joint_index": int, "value": float}  // 单关节增量移动
3. {"action": "hand_set_angles", "angles": [a1,...,a6]}  // 手指角度
4. {"action": "hand_open"}
5. {"action": "hand_close"}
6. {"action": "wait", "seconds": float}  // 等待
7. {"action": "grasp_sequence"}  // 预定义完整抓取序列（仅作后备）
注意：灵巧手角度范围 0~1000，通常张开用 [999,999,999,999,999,100]，闭合用 [50,50,50,50,900,100]。
机械臂关节角度依据你的观察位置调整。请根据任务生成合理序列。
"""

SYSTEM_PROMPT = f"""你是一个机器人控制专家。请将用户的自然语言指令转换为精确的动作基元序列。
只返回 JSON 数组，不要解释。{ACTION_PRIMITIVES}"""

def parse_command(user_text):
    """调用 LLM 并返回动作列表"""
    client = openai.OpenAI(api_key="your-api-key")  # 配置你的 key
    response = client.chat.completions.create(
        model="gpt-4o",  # 或其他
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_text}
        ],
        temperature=0.1
    )
    content = response.choices[0].message.content
    # 简单提取 JSON 部分（去除可能的 markdown 标记）
    import json, re
    json_str = re.sub(r'```(?:json)?', '', content).strip()
    actions = json.loads(json_str)
    return actions