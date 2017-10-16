# Dynamic A\* Lite
D\* Lite is an incremental heuristic search algorithm that builds on LPA\*, an incremental heuristic search algorithm that combines ideas of A\* and Dynamic SWSF-FP algorithm for single-shot grid-based 2D path finding. 

## Description
Current project provides an implementation of [D\*Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) algorithm adapted for single-shot grid-based 2D environment. 

Current implementation use single-shot grid-based 2D environment to build path from start cell to goal cell. For each iteration algorithm knows only about part of the environment that has been discovered, that consists of all cells that was or is in the  direct line of sight of travelling agent.
Line of sight of a travelling agent defines by radius. 
Algorithm uses LPA\* to build path depending on the current knowledge about the surroundings, then, after the move and corresponded cell changes algorithm rebuilds part of the path according to the changes.

Algorithm supports XML files as input and output format. Input file contains map and environment representation (see *"Input and Output files"* or [samples](https://bitbucket.org/nsoboleva2/d-lite/src/42e3e490eab6b0b4322796bd4f92e5667c93ae91/maps/?at=master))

## Getting Started 

To go and try this algorithm you can use QtCreator or CMake. 
Both `.pro` and `CMakeLists` files are available in the repository.

Notice, that project uses C++11 standart. Make sure that your compiler supports it.

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[Qt Creator](https://info.qt.io/download-qt-for-device-creation?hsCtaTracking=c80600ba-f2ea-45ed-97ef-6949c1c4c236%7C643bd8f4-2c59-4c4c-ba1a-4aaa05b51086)** -  a cross-platform C++, JavaScript and QML integrated development environment which is part of the SDK for the Qt GUI Application development framework. 

**[CMake](https://cmake.org/)** - an open-source, cross-platform family of tools designed to build, test and package software.

### Installing

Download current repository to your local machine. Use
```
git clone https://nsoboleva2@bitbucket.org/nsoboleva2/d-lite.git
```
or direct downloading.

Built current project using **Qt Creator** or **CMake**. To launch the compiled file you will need to pass input XML file as an argument. Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe, 
```
"initial_file_name.xml" -> "initial_file_name_log.xml"
```
For more detailed information there are some samples in the  [samples](https://bitbucket.org/nsoboleva2/d-lite/src/42e3e490eab6b0b4322796bd4f92e5667c93ae91/maps/?at=master) folder.

## Input and Output files

Both files are an XML file with a specific structure. 
Input file should contain:

* Mandatory tag `<map>`. It describes the environment.
    * `<height>` and `<width>` - mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width*-1, *height*-1) is lower right.
    * `<startx>` and `<starty>` - mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width*-1], for *starty* - [0, .., *height*-1].
    * `<finishx>` and `<finishy>` - mandatory tags that horizontal (X) and vertical (Y) offset of the goal location.
    * `<grid>` - mandatory tag that describes the square grid constituting the map. It consists of `<row>` tags. Each `<row>` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" - for untraversable (actually any other figure but "0" can be used instead of "1").
    * `<cellsize>` - optional tag that defines the size of one cell. One might add it to calculate scaled length of the path.
    * `<title>`, `<URL>`, `<coordinates>`, etc - optional tags containing additional information on the map.

* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.

    * `<metrictype>` - defines the type of metric for heuristic function. Possible values - "euclidean", "diagonal", "manhattan", "chebyshev". Default value is "euclidean".
    * `<hweight>` - defines the weight of heuristic function. Default value is "1".
    * `<radius>` - defines rational number - radius of agent's line of sight. Default value is "5".
    * `<allowdiagonal>` - boolean tag that defines the possibility to make diagonal moves. Setting it to "false" restricts agent to make cardinal (horizonal, vertical) moves only. Default value is "true".
    * `<cutcorners>` - boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. The tag is ignored if diagonal moves are not allowed. Default value is "false".
    * `<allowsqueeze>` - boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. The tag is ignored if cutting corners is not allowed. Default value is "false".
	* `<allowjump>` - boolean tag that defines the possibility to jump directly to the edge of line of sight after the LPA\* calculation. Default value is "false".

* Optional tag `<options>`. Options that are not related to search.

    * `<loglevel>` - defines the level of detalization of log-file. Default value is "1". Possible values:
        - "0" or "none" - log-file is not created.
        - "0.5" or "tiny" - All the input data is copied to the log-file plus short `<summary>` is appended. `<summary>` contains info of the path length, number of steps, elapsed time, etc.
        - "1" or "short" - *0.5*-log plus `<path>` is appended. It looks like `<grid>` but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: `<hplevel>` and `<lplevel>`. `<lplevel>` is the sequence of coordinates of cells forming the path. `<hplevel>` is the sequence of sections forming the path.
    * `<logpath>` - defines the directory where the log-file should be written. If not specified directory of the input file is used. 
    * `<logname>` - defines the name of log-file. If not specified the name of the log file is: "input file name"+"_log"+input file extension.

The main tag in Output file, which containes path length, memory and time:  
```xml
<summary numberofsteps="392" nodescreated="239" length="37.071068" length_scaled="100.09188308715821" time="0.011694"/>
```
* _"numberofsteps"_ stands for the number of iterations (number of expanded vertices)
* _"nodescreated"_  stands for the number of nodes that were examined in general (= memory)
* _"length"_ stands for length of the final path
* _"length_scaled"_ stands for actual length counting the size of cell in map
* _"time"_ stands for elapsed time