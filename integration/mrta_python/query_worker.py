"""Private bounded-query subprocess; stdin is only from the trusted planner.

Do not expose this pickle channel as a network or user-file interface.
"""
import contextlib
import pickle
import sys
import struct


def main():
    stream='--stream' in sys.argv
    output=sys.stdout.buffer
    def send(response):
        data=pickle.dumps(response)
        if stream:output.write(struct.pack('!Q',len(data)))
        output.write(data);output.flush()
    try:
        provider,arguments=pickle.load(sys.stdin.buffer)
        with contextlib.redirect_stdout(sys.stderr):
            result=provider(*arguments)
            if stream:
                for candidate in result:send((True,candidate))
                return
        response=(True,result)
    except Exception as error:
        response=('BUDGET' if stream and type(error).__name__=='PlanningBudgetExceeded' else False,str(error))
    send(response)


if __name__=='__main__':
    main()
