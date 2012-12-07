#! /usr/bin/env python
import rospy
import message_filters
import threading
import itertools

class ApproximateSynchronizer(message_filters.SimpleFilter):

    def __init__(self, slop, fs, queue_size):
        message_filters.SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()
        self.slop = rospy.Duration.from_sec(slop)

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [f.registerCallback(self.add, q) for (f, q) in zip(fs, self.queues)]

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        if 0:
            # common is the set of timestamps that occur in all queues
            common = reduce(set.intersection, [set(q) for q in self.queues])
            for t in sorted(common):
                # msgs is list of msgs (one from each queue) with stamp t
                msgs = [q[t] for q in self.queues]
                self.signalMessage(*msgs)
                for q in self.queues:
                    del q[t]
        else:
            for vv in itertools.product(*[q.keys() for q in self.queues]):
                if ((max(vv) - min(vv)) < self.slop):
                    msgs = [q[t] for q,t in zip(self.queues, vv)]
                    self.signalMessage(*msgs)
                    for q in self.queues:
                        try:
                            del q[t]
                        except KeyError:
                            pass # TODO: why can del q[t] fail?
        self.lock.release()
