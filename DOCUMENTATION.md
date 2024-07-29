
## Docker setup
* Run below commands to setup docker container for development: 
* docker build -t milrem .
* docker run -it -v ${PWD}:/milrem milrem /bin/bash
* docker exec -it `<containerhash>` /bin/bash

## VSCode debug 
* Setup in .vscode folder 
* Launch with gdbserver and attach to pid via gdb 

# Some thoughts and things:
* Packet.hpp is a shortcut to just store somehow, the data perhaps could be dynamically allocated in a way 2 header bytes +     payload right after?  
    struct header {
        uint8_t sensorId;
        uint8_t dataType;
        // payload follows
    }
  It would be helpful to clarify what is meant by "Data must be stored in the type defined in UDP packet." I assumed there's some effective way for allocation. For testing used a C-style union, which is not the most space-efficient.
* Not sure if Boost library was allowed/meant to be used(for instance for UDP communication?) - myself did not use.
* Average (mean) data TODO - Would be good to know which concept would be good to use - I was thinking of calculating all packets using hashmap for statistical data in one Task and then reset stats every N second in publish callback - thread safety must be considered.
* For Logger node perhaps std queue or ringbuffer.hpp could have been used(thus created common package for it) 
* Modified build.sh to be able to turn off optimization and enable debug symbols, and integrate with VSCode toolset.
* In general - nice challenge! Quite extensive, reached time limit 6h. Perhaps these concepts can be rather conceptually tested or on the whiteboard?  
