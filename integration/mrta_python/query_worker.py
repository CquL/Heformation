"""Private bounded-query subprocess; stdin is only from the trusted planner.

Do not expose this pickle channel as a network or user-file interface.
"""
import contextlib
import pickle
import sys


def main():
    try:
        provider,arguments=pickle.load(sys.stdin.buffer)
        with contextlib.redirect_stdout(sys.stderr):
            result=provider(*arguments)
        response=(True,result)
    except Exception as error:
        response=(False,str(error))
    pickle.dump(response,sys.stdout.buffer)
    sys.stdout.buffer.flush()


if __name__=='__main__':
    main()
