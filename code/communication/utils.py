# 推进器布局
# 前进方向
# 6   3
# 4   1
# 5   2
ref = 1500
delta = 500
pos_val = ref + delta
neg_val = ref - delta
keys = ['w', 's', 'a', 'd', 'q', 'e', 'v']
# 将按键转换为PWM值
def key2cmd(key):
    pwm_values = [ref, ref, ref, ref, ref, ref]  # 初始化PWM值
    
    if key == 'w':
        pwm_values = [ref, pos_val, pos_val, ref, pos_val, pos_val]
    elif key == 's':
        pwm_values = [ref, neg_val, neg_val, ref, neg_val, neg_val]
    elif key == 'a':
        pwm_values = [ref, neg_val, pos_val, ref, pos_val, neg_val]
    elif key == 'd':
        pwm_values = [ref, pos_val, neg_val, ref, neg_val, pos_val]
    elif key == 'q':
        pwm_values = [ref, neg_val, neg_val, ref, pos_val, pos_val]
    elif key == 'e':
        pwm_values = [ref, pos_val, pos_val, ref, neg_val, neg_val]
    elif key == 'v':
        pwm_values = [ref, ref, ref, ref, ref, ref]
    # 将PWM值转换为字符串格式
    pwm_str = "PWM:" + ','.join(map(str, pwm_values))
    
    return pwm_str