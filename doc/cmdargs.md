## <a name="main_help"></a> rostopicmon.py --help
```
usage: rostopicmon.py [-h] [--listtools] [-la] {list,raw,stats} ...

ROS topic measurement tools. Collect size of messages passed through topics
and calculate various statistics. All data is measured in Bytes, Secounds and
Hertz accordingly.

optional arguments:
  -h, --help        show this help message and exit
  --listtools       List tools
  -la, --logall     Log all messages

subcommands:
  use one of tools

  {list,raw,stats}  one of tools
    list            list topics
    raw             collect and store raw data (sizes of messages)
    stats           collect and store stats data
```



## <a name="list_help"></a> rostopicmon.py list --help
```
usage: rostopicmon.py list [-h]

list topics

optional arguments:
  -h, --help  show this help message and exit
```



## <a name="raw_help"></a> rostopicmon.py raw --help
```
usage: rostopicmon.py raw [-h] [--topic N [N ...]] [--duration DURATION]
                          [--nosummary] [--calcstddev] [-la]
                          [--outfile OUTFILE] [--outdir OUTDIR]
                          [--outformat {json,csv,xls,xlsx}]

collect and store raw data (sizes of messages)

optional arguments:
  -h, --help            show this help message and exit
  --topic N [N ...]     Space separated list of regex strings applied on found
                        topics to listen on. It can also be paths to files
                        with regexes. Or mix of both. Example: "--topic
                        '/turtle1/.*' '/ros.*'"
  --duration DURATION   Set monitor time in seconds. Stop application after
                        timeout.
  --nosummary           Do not generate topics summary.
  --calcstddev          Calculate standard deviation (time consuming).
  -la, --logall         Log all messages
  --outfile OUTFILE     Path to output file (store collected data in single
                        file).
  --outdir OUTDIR       Path to output dir (store collected data in
                        directory).
  --outformat {json,csv,xls,xlsx}
                        Output format. Default: json.
```



## <a name="stats_help"></a> rostopicmon.py stats --help
```
usage: rostopicmon.py stats [-h] [-w WINDOW] [--nostoreraw]
                            [--fromrawfile FROMRAWFILE] [--topic N [N ...]]
                            [--duration DURATION] [--nosummary] [--calcstddev]
                            [-la] [--outfile OUTFILE] [--outdir OUTDIR]
                            [--outformat {json,csv,xls,xlsx}]

collect and store stats data

optional arguments:
  -h, --help            show this help message and exit
  -w WINDOW, --window WINDOW
                        Set window size, otherwise collect all samples.
  --nostoreraw          Do not store raw data additionally.
  --fromrawfile FROMRAWFILE
                        Path to raw file to get data from.
  --topic N [N ...]     Space separated list of regex strings applied on found
                        topics to listen on. It can also be paths to files
                        with regexes. Or mix of both. Example: "--topic
                        '/turtle1/.*' '/ros.*'"
  --duration DURATION   Set monitor time in seconds. Stop application after
                        timeout.
  --nosummary           Do not generate topics summary.
  --calcstddev          Calculate standard deviation (time consuming).
  -la, --logall         Log all messages
  --outfile OUTFILE     Path to output file (store collected data in single
                        file).
  --outdir OUTDIR       Path to output dir (store collected data in
                        directory).
  --outformat {json,csv,xls,xlsx}
                        Output format. Default: json.
```
