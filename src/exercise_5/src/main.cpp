#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Point {
 public:
  float x;
  float y;

  Point() : x(0), y(0) {}
  Point(float x_, float y_) : x(x_), y(y_) {}

  static Point fromPolar(float radius, float angle) {
    float x_ = radius * std::cos(angle);
    float y_ = radius * std::sin(angle);

    return Point(x_, y_);
  }

  float distance(const Point& other) const {
    float a = other.x - x;
    float b = other.y - y;

    return std::sqrt(a * a + b * b);
  }

  Point operator+(const Point& other) const {
    float x_ = x + other.x;
    float y_ = y + other.y;

    return Point(x_, y_);
  }

  Point operator-(const Point& other) const {
    float x_ = x - other.x;
    float y_ = y - other.y;

    return Point(x_, y_);
  }

  Point operator*(const float& factor) const {
    float x_ = x * factor;
    float y_ = y * factor;

    return Point(x_, y_);
  }

  Point operator/(const float& factor) const {
    float x_ = x / factor;
    float y_ = y / factor;

    return Point(x_, y_);
  }
};

std::ostream& operator<<(std::ostream& os, const Point& point) {
  os << "(" << point.x << ", " << point.y << ")";
}

class Leg {
 private:
  std::vector<Point> points;

  void calculateCenter() {
    Point temp;
    for (size_t i = 0; i < points.size(); i++) {
      temp = temp + points[i];
    }

    center = temp / points.size();
  }

 public:
  Point center;

  void addPoint(const Point point) {
    points.push_back(point);
    calculateCenter();
  }

  float distance(const Leg& leg) const {
    return center.distance(leg.center);
  }
};

std::ostream& operator<<(std::ostream& os, const Leg& leg) {
  os << "Center of leg: " << leg.center;
}

class Person {
 private:
  std::vector<Leg> legs;

  void calculateCenter() {
    Point temp;
    for (size_t i = 0; i < legs.size(); i++) {
      temp = temp + legs[i].center;
    }

    center = temp / legs.size();
  }

 public:
  Point center;

  bool addLeg(const Leg leg) {
    if (legs.size() == 2) {
      return false;
    }

    legs.push_back(leg);

    calculateCenter();

    return true;
  }
};

std::ostream& operator<<(std::ostream& os, const Person& person) {
  os << "Center of person: " << person.center;
}

class PeopleDetector {
 private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber scanSubscriber;

  std::vector<float> data;
  float angleIncrement;

  int thresholdLeg;
  float thresholdPerson;

  std::vector<Leg> legs;
  std::vector<Person> people;

  void callback(sensor_msgs::LaserScanConstPtr msg) {
    angleIncrement = msg->angle_increment;
    data = msg->ranges;

    legs.clear();
    people.clear();

    calculateLegs();
    calculatePeople();
  }

  void calculateLegs() {
    int count = 0;

    legs.push_back(Leg());

    for (size_t i = 0; i < data.size(); i++) {
      if (data[i] != INFINITY) {
        Point point = Point::fromPolar(data[i], angleIncrement * i);

        if (count < thresholdLeg) {
          legs.back().addPoint(point);
        } else {
          legs.push_back(Leg());
          legs.back().addPoint(point);
        }

        count = 0;
      } else {
        count++;
      }
    }

    ROS_INFO("There are %d legs", legs.size());
  }

  void calculatePeople() {
    int step = 1;
    for (size_t i = 0; i < legs.size() - 1; i += step) {
      people.push_back(Person());

      people.back().addLeg(legs[i]);

      int previousIndex = (i + legs.size() - 1) % legs.size();
      if (legs[i].center.distance(legs[previousIndex].center) < thresholdPerson) {
        people.back().addLeg(legs[previousIndex]);
        step = 2;
      } else {
        step = 1;
      }

      int nextIndex = i + 1;
      if (legs[i].center.distance(legs[nextIndex].center) < thresholdPerson) {
        if (people.back().addLeg(legs[nextIndex])) {
          step = 2;
        } else {
          step = 1;
        }
      } else {
        step = 1;
      }
    }

    ROS_INFO("There are %d people", people.size());

    for (size_t i = 0; i < people.size(); i++) {
      ROS_INFO_STREAM(people[i]);
    }
  }

 public:
  PeopleDetector(int thresholdLeg_, float thresholdPerson_) : nodeHandle(ros::NodeHandle()), thresholdLeg(thresholdLeg_), thresholdPerson(thresholdPerson_) {
    scanSubscriber = nodeHandle.subscribe("scan", 1000, &PeopleDetector::callback, this);

    ROS_INFO("Init");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "people_detector");

  PeopleDetector p = PeopleDetector(5, 0.7);

  ros::spin();
  return 0;
}