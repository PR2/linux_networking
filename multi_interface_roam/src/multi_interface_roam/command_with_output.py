#! /usr/bin/env python

import threading
import logging
import logging.handlers
import os
from logging_config import *
import subprocess
import fcntl
import time
import traceback
import signal
import select

class CommandWithOutput(threading.Thread):
    def __init__(self, args, name):
        self.restart_delay = 0.2
        threading.Thread.__init__(self, name = name)
        #logname = os.path.join(logdir, '%s.log'%name)
        #try:
        #    os.makedirs(logdir)
        #except OSError, e:
        #    if e.errno != errno.EEXIST:
        #        raise
        #print "Creating log file:", logname
        #self.log = open(os.path.join(logname), 'a')
        #self.log.write("\n\n\nStarting new session...\n")
        self.logger = logging.getLogger(name)
        self.console_logger = logging.getLogger('console.%s'%name)
        logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'%s.log'%name), when='midnight', backupCount=logfilecount)
        logger_handler.setFormatter(file_formatter)
        self.logger.addHandler(logger_handler)
        self.console_logger.addHandler(logger_handler)
        self.logger.setLevel(logging.DEBUG)
        self.console_logger.setLevel(logging.DEBUG)
        logger_handler.setLevel(logging.DEBUG)
        self.proc_args = args
        self.start_proc()
        self.running = True
        self.start()
                            
    def start_proc(self):
        try:
            self.proc = subprocess.Popen(self.proc_args, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, close_fds = True)
            flags = fcntl.fcntl(self.proc.stdout, fcntl.F_GETFL)
            fcntl.fcntl(self.proc.stdout, fcntl.F_SETFL, flags| os.O_NONBLOCK)
            self.child_restart()
        except OSError:
            self.console_logger.fatal("Error trying to run: %s"%(" ".join(self.proc_args)))
            raise KeyboardInterrupt

    def child_restart(self):
        pass # Can be overridden by derived classes.

    def run(self):
        read_buffer = {}
        try:
            while True:
                (rd, wr, err) = select.select([self.proc.stdout], [], [], 0.2)
                if not self.running:
                    #print "Exiting CommandWithOutput", self.proc.pid, self.proc_args
                    try:
                        self.proc.send_signal(signal.SIGINT)
                    except OSError, e:
                        if str(e).find('[Errno 3]') == -1:
                            raise
                    #print "Starting Communicate", self.proc_args
                    try:
                        self.proc.communicate()
                    except IOError:
                        pass
                    #print "Ending Communicate", self.proc_args
                    return
                for fd in rd:
                    #print >> sys.stderr, "About to read"
                    try:
                        newdata = fd.read()
                    except IOError:
                        newdata = ""
                    #print >> sys.stderr, "Done read", newdata
                    if len(newdata) == 0: 
                        self.proc.kill()
                        self.proc.communicate()
                        time.sleep(self.restart_delay)
                        if not self.running: 
                            return
                        self.console_logger.info("Process died, restarting: %s"%(" ".join(self.proc_args)))
                        self.start_proc()
                        continue

                    if fd in read_buffer:
                        newdata = read_buffer[fd] + newdata 
                    while True:
                        splitpos = newdata.find('\n')
                        if splitpos == -1:
                            read_buffer[fd] = newdata
                            break
                        line = newdata[0:splitpos]
                        newdata = newdata[splitpos+1:]

                        self.logger.info(line)
                        #now = time.time()
                        #time_str = log_time_string(now)
                        #self.log.write(time_str+": "+line+"\n")
                        #self.log.flush()
                        #sys.stdout.write("%s %s: %s"%(time_str, self.name, line))
                        #sys.stdout.flush()
                        try:
                            self.got_line(line)
                        except Exception, e:
                            self.console_logger.fatal("Caught exception in CommandWithOutput.run: %s"%str(e))
                            raise # FIXME Remove this?
        except: # FIXME Be more persistent?
            traceback.print_exc(10)
            print
            self.console_logger.fatal("Command with output triggering shutdown after exception.")
            os.kill(os.getpid(), signal.SIGINT)
            raise

    def shutdown(self):
        self.running = False
        #print "Shutting down command with output:", self.proc.pid, self.proc_args
        #try:
        #    self.proc.kill()
        #    print "Process killed", self.proc_args
        #except OSError:
        #    print "Process already dead", self.proc_args

