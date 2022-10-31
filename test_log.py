from os.path import join

from contextlib import contextmanager
import sys, os

@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout


class writeScanlog:
    
    def default_log(rotation):
        setup_log = ["rotation\t: " + str(rotation), "distance\t: " + str(0), "radius\t\t: " + str(0), "disparity\t: " + str(0)]
        print(setup_log)
        with open(join("dataset", "log.txt"), 'w') as f:
            f.write('\n'.join(setup_log))
        f.close()
        print("default log created")
        
            
    def custom_log(rotation, distance, radius, disparity):
        setup_log = ["rotation\t: " + str(rotation), "distance\t: " + str(distance), "radius\t\t: " + str(radius), "disparity\t: " + str(disparity)]
        print(setup_log)
        with open(join("dataset", "log.txt"), 'w') as f:
            f.write('\n'.join(setup_log))
        f.close()
        print("custom log created")
            
            
def write_reconstruction_log(make, register, refine, integrate):
    with open(join("dataset", "log.txt"), 'a') as f:
        f.write("\n\nMaking fragments    %s" % make)
        f.write("\nRegister fragments  %s" % register)
        f.write("\nRefine registration %s" % refine)
        f.write("\nIntegrate frames    %s" % integrate)
        f.write("\nTotal               %s" % str(make+register+refine+integrate))        
    f.close()
    
    


rotation, distance, radius, disparity = None, None, None, None

rotation = 6

distance = 100
radius = 10
disparity = 50

make, register, refine, integrate = 10, 20, 30, 40
    
print("You can see this")
with suppress_stdout():
    print("You cannot see this")
    if (distance or radius or disparity) != None:
        writeScanlog.custom_log(rotation, distance, radius, disparity)
    else:
        writeScanlog.default_log(rotation)
print("And you can see this again")

write_reconstruction_log(make, register, refine, integrate)