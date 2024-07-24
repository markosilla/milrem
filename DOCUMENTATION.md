
## Docker setup
* docker build -t milrem .
* docker run -it -v ${PWD}:/milrem milrem /bin/bash
* docker exec -it `<containerhash>` /bin/bash
