# ros-topic-monitor

Basic functionality of the project resembles those of [rostopic](http://wiki.ros.org/rostopic) command-line tool.

Features:
- listen on multiple topics
- collect raw data (sizes of messages)
- calculate min, max, mean and stddev
- calculate stats based on all data or in window of given size
- store output in following formats: json, csv, xls and xlsx

Description of command-line arguments can be found [here](doc/cmdargs.md).


## Installation

Installation of package can be done by:
 - to install package from downloaded ZIP file execute: `pip3 install --user file:ros-topic-monitor-master.zip#subdirectory=src`
 - to install package directly from GitHub execute: `pip3 install --user git+https://github.com/anetczuk/ros-topic-monitor.git#subdirectory=src`
 - uninstall: `pip3 uninstall --user ros-topic-monitor`

Installation For development:
 - `install-deps.sh` to install package dependencies only (`requirements.txt`)
 - `install-package.sh` to install package in standard way through `pip` (with dependencies)
 - `install-package-dev.sh` to install package in developer mode using `pip` (with dependencies)


## Running application

To run application try one of:
- `rostopicmon.py --help`
- `python3 -m rostopicmonitor --help`

to receive [cmd help](doc/cmdargs.md).

To grab stats from all topics simply run `python3 -m rostopicmonitor stats --outdir outdata`

Running preselected topics goes like this: `python3 -m rostopicmonitor stats --outdir outdata --topic '/turtle1/.*'` 
In this case `turtlesim` topics will be monitored.

For more advanced usage see [help](doc/cmdargs.md).


## Units

Uses following units:
- size: *Byte*,
- time: *Second*,
- frequency: *Hertz*,
- bandwidth: *Bytes per Second*


## References

- [rostopic command-line tool](http://wiki.ros.org/rostopic)


## License

BSD 3-Clause License

Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
