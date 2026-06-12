#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <opencv2/opencv.hpp>

struct Node
{
  int id;
  cv::Point pos; // averaged position after merging
  bool is_endpoint;
};

struct Edge
{
  int src, dst;                // node IDs
  std::vector<cv::Point> path; // pixel chain along the skeleton
  double length;               // Euclidean arc length

  bool operator==(const Edge & other) const
  {
    return (src == other.src && dst == other.dst) ||
           (src == other.dst && dst == other.src);
  }
};

struct Graph
{
  std::vector<Node> nodes;
  std::vector<Edge> edges;
};

enum NavigationType
{
  SIMPLE,
  ADVANCED
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NavigationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  NavigationNode();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  int pathLimit;
  int minEdgeSize;
  NavigationType navigationType;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> errorPub;

  void simpleNavigation(cv::Mat & frame);
  void advancedNavigation(cv::Mat & frame);

  void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat processImage(cv::Mat & image);
  cv::Mat getGreen(cv::Mat & image);

  void publishError(double error);

  std::vector<Node> findPath(Node startPos);

  void extractNodes();
  void extractEdges();

  std::vector<cv::Point> getSurroundingPoints(cv::Point centre, int radius);
  std::vector<Edge> traceConnectedEdges(Node node);
  Node followToNode(std::vector<cv::Point> & path);

  void removeShortEdges(std::vector<Edge> & edges);
  Edge mergeEdges(Edge edge1, Edge edge2);

  void findNextNode(std::vector<Node> & path);
  double calculateAngle(cv::Point point1, cv::Point point2);

  Node * nodeFromID(int id);
  std::vector<Edge *> getConnectedEdges(int nodeID);
  std::vector<double> getEdgeDirections(Node origin, std::vector<Edge *> edges);

  Graph graph;
};
