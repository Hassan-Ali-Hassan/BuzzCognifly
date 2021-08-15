## About

BuzzCognifly is a [Buzz](http://the.swarming.buzz/) implementation for the Cognifly quadcopter.

### Prerequisites

The following instructions are made  under the assumption that the installation will take place on the target operating computer (e.g. onboard computer) directly. The following steps have been tried with a Raspberry Pi running Raspbian Buster 10, but similar prerequisites will apply for different versions of linux and different onboard computers.

* [Install ASIO] ```sudo apt install --no-install-recommends libasio-dev```
* Make sure to install gcc-multilib:

```
sudo apt-get install gcc-multilib
```

To have a cleaner workspace, it is suggested to creata folder, e.g. khepera-proper, and inside, clone:
* [Buzz](https://github.com/MISTLab/Buzz)
* [BuzzCognifly]

## Installation

In order to sucessfully compile the project, you need a cross compiler for Yocto linux and [Khepera IV](https://www.k-team.com/khepera-iv) API.


1. We start by installing [Buzz]. In the [Buzz] folder:

```
mkdir build
cd build
cmake ../src/
sudo make install
```

2. Similarly, we install BuzzCognifly:

```
mkdir build
cd build
cmake ../src/
sudo make
```

4. The latter process produces a filled called bzzCognifly. This is the file we use to run Buzz code on the cogilfy, for example:

```
./bzzCognifly tcp 500 test_case.bo test_case.bdb
```
where ```test_case.*``` result from compiling the Buzz code: ```bzzc test_case.bzz```. 



