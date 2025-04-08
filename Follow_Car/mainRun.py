# import relevant packages
import multiprocessing
from multiprocessing import Queue
from control import *
from processImage import *
import time

if __name__ == "__main__": 

    # create queues to pass information between subprocesses
    frame_queue = Queue()
    position_queue = Queue()
    exit_flag = Queue()

    # create a subprocess for the YOLO model processing and control of vehicle components
    p1 = multiprocessing.Process(target=process_new_frame, args = (frame_queue, position_queue))
    p2 = multiprocessing.Process(target=drive, args = (frame_queue, position_queue, exit_flag))

    try:
        # start the subproccesses
        print("Starting processes...")
        p1.start()
        p2.start()

        # Keep parent proccess alive while subprocceses run
        while True:
            time.sleep(5)

    except (KeyboardInterrupt, Exception) as e: # exit by keyboard interrupt
        print(f"Exception caught: {e}. Terminating processes...")

    finally:
        exit_flag.put(1) # inform subprocess of ending parent process
        time.sleep(1) # wait for system to stop

        # cleanly end processes
        p1.terminate()
        p2.terminate()
        p1.join()
        p2.join()
        print("Processes terminated cleanly.")
