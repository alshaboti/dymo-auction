import lcm

from commsg import comm_msg_t

def my_handler(channel, data):
    msg = comm_msg_t.decode(data)
    print msg.sender," ", msg.receiver, " ", msg.msg_type, " R: ", msg.r, " Q: ", msg.q, " SEQ: ", msg.seq_no

lc = lcm.LCM()
subscription = lc.subscribe("TATopic", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
