/*
NAME : DIYAA YADAV
SID : 1711060
CCID : DIYAA
ANONYMOUS ID : 1000319371
COURSE: CMPUT 275
TERM : WINTER 2023
ASSIGNMENT PART 2
*/
#include <iostream>
#include <cassert>
#include <fstream>
#include <string>
#include <list>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "wdigraph.h"
#include "dijkstra.h"

struct Point {
    long long lat, lon;
};

// return the manhattan distance between the two points
long long manhattan(const Point& pt1, const Point& pt2) {
  long long dLat = pt1.lat - pt2.lat, dLon = pt1.lon - pt2.lon;
  return abs(dLat) + abs(dLon);
}

// find the ID of the point that is closest to the given point "pt"
int findClosest(const Point& pt, const unordered_map<int, Point>& points) {
  pair<int, Point> best = *points.begin();

  for (const auto& check : points) {
    if (manhattan(pt, check.second) < manhattan(pt, best.second)) {
      best = check;
    }
  }
  return best.first;
}

// read the graph from the file that has the same format as the "Edmonton graph" file
void readGraph(const string& filename, WDigraph& g, unordered_map<int, Point>& points) {
  ifstream fin(filename);
  string line;

  while (getline(fin, line)) {
    // split the string around the commas, there will be 4 substrings either way
    string p[4];
    int at = 0;
    for (auto c : line) {
      if (c == ',') {
        // start new string
        ++at;
      }
      else {
        // append character to the string we are building
        p[at] += c;
      }
    }

    if (at != 3) {
      // empty line
      break;
    }

    if (p[0] == "V") {
      // new Point
      int id = stoi(p[1]);
      assert(id == stoll(p[1])); // sanity check: asserts if some id is not 32-bit
      points[id].lat = static_cast<long long>(stod(p[2])*100000);
      points[id].lon = static_cast<long long>(stod(p[3])*100000);
      g.addVertex(id);
    }
    else {
      // new directed edge
      int u = stoi(p[1]), v = stoi(p[2]);
      g.addEdge(u, v, manhattan(points[u], points[v]));
    }
  }
}

int create_and_open_fifo(const char * pname, int mode) {
  // create a fifo special file in the current working directory with
  // read-write permissions for communication with the plotter app
  // both proecsses must open the fifo before they perform I/O operations
  // Note: pipe can't be created in a directory shared between 
  // the host OS and VM. Move your code outside the shared directory
  if (mkfifo(pname, 0666) == -1) {
    cout << "Unable to make a fifo. Make sure the pipe does not exist already!" << endl;
    cout << errno << ": " << strerror(errno) << endl << flush;
    exit(-1);
  }

  // opening the fifo for read-only or write-only access
  // a file descriptor that refers to the open file description is
  // returned
  int fd = open(pname, mode);

  if (fd == -1) {
    cout << "Error: failed on opening named pipe." << endl;
    cout << errno << ": " << strerror(errno) << endl << flush;
    exit(-1);
  }

  return fd;
}

// keep in mind that in part 1, the program should only handle 1 request
// in part 2, you need to listen for a new request the moment you are done
// handling one request
/*
Description : 
    function's objective is to take the coordinate from inpipe, then parse
    through the input data and identify the closest points using the dijkstra's
    algorithm. Finally, the computed results are written to the outpipe. 
Arguments : 
    no arguments
Return : 
    returns nothing
*/
int main() {
  WDigraph graph;
  unordered_map<int, Point> points;

  const char *inpipe = "inpipe";
  const char *outpipe = "outpipe";

  // Open the two pipes
  int in = create_and_open_fifo(inpipe, O_RDONLY);
  cout << "inpipe opened..." << endl;
  int out = create_and_open_fifo(outpipe, O_WRONLY);
  cout << "outpipe opened..." << endl;  

  // build the graph
  readGraph("server/edmonton-roads-2.0.1.txt", graph, points);

  // read a request
  char currentbuffer;
  string tempstore = "";
  Point sPoint, ePoint;       // start and end points
  bool iterate = true;
  int datapointcount = 0; 
  vector<long long> storeofpoints;

  while (iterate){
    // read the input buffer
    while (true){
      // read the current byte
      read(in, &currentbuffer, 1); 

      // check if the byte read is a newline character
      if (currentbuffer == '\n'){
        storeofpoints.push_back(static_cast<long long>(stod(tempstore)*100000));
        tempstore = "";
        datapointcount++;
        // we break the loop to read the next byte
        break;
      }
      else if(currentbuffer == 'Q'){
        iterate = true;
        close(in);
        unlink(inpipe);
        close(out);
        unlink(outpipe);
        break;
      }
      if (currentbuffer == ' '){
        storeofpoints.push_back(static_cast<long long>(stod(tempstore)*100000));
        tempstore = "";
      }
      else{
        tempstore.push_back(currentbuffer);
      }
    }

    // checks if we have start point having latitude and longitude
    if (datapointcount < 2){
      sPoint.lat = storeofpoints[0];
      sPoint.lon = storeofpoints[1];
    }
    else{
      datapointcount = 0;
      ePoint.lat = storeofpoints[2];
      ePoint.lon = storeofpoints[3];

      storeofpoints.clear();
      // get the points closest to the two points we read
      int start = findClosest(sPoint, points), end = findClosest(ePoint, points);

      // run dijkstra's algorithm, this is the unoptimized version that
      // does not stop when the end is reached but it is still fast enough
      unordered_map<int, PIL> tree;
      dijkstra(graph, start, tree);

      // NOTE: in Part II you will use a different communication protocol than Part I
      // So edit the code below to implement this protocol

      // no path
      if (tree.find(end) == tree.end()) {
        write(out,"E\n", strlen("E\n"));
      }
      else {
        // read off the path by stepping back through the search tree
        list<int> path;
        while (end != start) {
          path.push_front(end);
          end = tree[end].first;
        }
        path.push_front(start);

        for (int v : path) {
          double latitudeOut = static_cast<double>((points[v].lat)), longitudeOut = static_cast<double>((points[v].lon)); 
          string sLat = to_string(latitudeOut/100000), sLon = to_string(longitudeOut/100000); //convert the long double to string

          sLat.pop_back();
          sLon.pop_back();

          string jointstring = sLat + " " + sLon;
          write(out, jointstring.c_str(), jointstring.size());
          write(out,"\n", strlen("\n"));
        }
        write(out,"E\n", strlen("E\n"));
      }
    }
  }
  return 0;
}