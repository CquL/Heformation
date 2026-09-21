// Bounded external caller owns the process deadline. This program only reads
// parameters and a declared static map snapshot; no Goal/trajectory publisher.
#include <ros/ros.h>
#include <optimizer/poly_traj_optimizer.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <fstream>
#include <set>

using boost::property_tree::ptree;

Eigen::Vector3d vector3(const ptree &tree) {
  if (tree.size()!=3) throw std::runtime_error("expected finite 3-vector");
  Eigen::Vector3d value; int i=0;
  for (const auto &v:tree) value(i++)=v.second.get_value<double>();
  if (!value.allFinite()) throw std::runtime_error("nonfinite vector");
  return value;
}
ptree values(const Eigen::VectorXd &v) {
  ptree out;
  for (int i=0;i<v.size();++i) {ptree x;x.put_value(v(i));out.push_back({"",x});}
  return out;
}

int main(int argc,char **argv) {
  if (argc<4) {std::cerr<<"usage: swarm_readonly_query source-namespace input.json output.json\n";return 2;}
  const std::string source=argv[1], input=argv[2], output=argv[3];
  ros::init(argc,argv,"swarm_readonly_query",ros::init_options::AnonymousName|ros::init_options::NoRosout);
  ptree result;result.put("status","UNKNOWN");
  try {
    ros::NodeHandle nh(source);
    XmlRpc::XmlRpcValue before,after;
    if (!nh.getParam("",before)) throw std::runtime_error("planner parameter namespace unavailable");
    double resolution,maximum_speed;int count,configured_id,formation_size;
    if (!nh.getParam("grid_map/resolution",resolution) || resolution<=0 ||
        !nh.getParam("optimization/max_vel",maximum_speed) || maximum_speed<=0 ||
        !nh.getParam("optimization/constrain_points_perPiece",count) || count<1 ||
        !nh.getParam("manager/drone_id",configured_id)) throw std::runtime_error("planner configuration incomplete");
    nh.param("optimization/formation_size",formation_size,7);
    ptree request;boost::property_tree::read_json(input,request);
    const int id=request.get<int>("drone_id");const bool formation=request.get<bool>("formation");
    if (id!=configured_id || id<0 || id>=formation_size) throw std::runtime_error("member/configuration mismatch");
    const auto start=vector3(request.get_child("start"));
    const auto velocity=vector3(request.get_child("velocity"));
    const auto acceleration=vector3(request.get_child("acceleration"));
    const auto target=vector3(request.get_child("target"));
    if ((target-start).norm()<1e-9) throw std::runtime_error("stationary target uses existing terminal method, not optimizer query");
    auto map=std::make_shared<GridMap>();map->initMap(nh,true);
    std::vector<Eigen::Vector3d> points;
    for (const auto &point:request.get_child("points")) points.push_back(vector3(point.second));
    map->loadStaticSnapshot(points);
    if (!map->isInMap(start) || !map->isInMap(target)) throw std::runtime_error("query endpoints outside configured map");
    if (map->getInflateOccupancy(start)==1 || map->getInflateOccupancy(target)==1) {
      result.put("status","INFEASIBLE");throw std::runtime_error("endpoint inside inflated declared obstacle");
    }
    ego_planner::SwarmTrajData peers(formation_size);
    for (auto &peer:peers) peer.drone_id=-1;
    std::set<int> supplied;
    for (const auto &entry:request.get_child("peers")) {
      const auto &raw=entry.second;const int peer_id=raw.get<int>("id");
      if (peer_id==id || peer_id<0 || peer_id>=formation_size || !supplied.insert(peer_id).second)
        throw std::runtime_error("invalid or duplicate peer identity");
      auto &peer=peers[peer_id];peer.drone_id=peer_id;peer.traj_id=0;
      std::vector<double> durations;std::vector<poly_traj::CoefficientMat> coefficients;
      if (raw.get<bool>("stationary",false)) {
        poly_traj::CoefficientMat c=poly_traj::CoefficientMat::Zero();c.col(5)=vector3(raw.get_child("position"));
        durations.push_back(1.);coefficients.push_back(c);peer.start_time=ros::Time::now().toSec();
      } else {
        peer.start_time=raw.get<double>("start_time");
        for (const auto &d:raw.get_child("durations")) {
          double duration=d.second.get_value<double>();
          if (!std::isfinite(duration) || duration<=0) throw std::runtime_error("invalid peer duration");
          durations.push_back(duration);
        }
        for (const auto &piece:raw.get_child("coefficients")) {
          if (piece.second.size()!=3) throw std::runtime_error("peer coefficients need 3 rows");
          poly_traj::CoefficientMat c;int row=0;
          for (const auto &axis:piece.second) {
            if (axis.second.size()!=6) throw std::runtime_error("peer coefficient order must be 5");
            int col=0;for (const auto &v:axis.second) c(row,col++)=v.second.get_value<double>();++row;
          }
          if (!c.allFinite()) throw std::runtime_error("invalid peer coefficients");
          coefficients.push_back(c);
        }
      }
      if (durations.empty() || durations.size()!=coefficients.size()) throw std::runtime_error("incomplete peer trajectory");
      peer.traj=poly_traj::Trajectory(durations,coefficients);peer.duration=peer.traj.getTotalDuration();
    }
    if (supplied.size()!=static_cast<size_t>(formation_size-1)) throw std::runtime_error("all peer references must be declared");
    ego_planner::PolyTrajOptimizer optimizer;
    optimizer.setParam(nh);optimizer.setDroneId(id);optimizer.setEnvironment(map);optimizer.setSwarmTrajs(&peers);
    Eigen::Matrix3d head,tail;head<<start,velocity,acceleration;tail<<target,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
    poly_traj::MinJerkOpt initial;Eigen::MatrixXd control;std::vector<Eigen::Vector3d> path;
    if (!optimizer.astarWithMinTraj(head,tail,path,control,initial)) throw std::runtime_error("native initial path unavailable");
    optimizer.setControlPoints(control);
    auto guess=initial.getTraj();auto positions=guess.getPositions();
    const double reference_start=request.get<double>("reference_start_time",ros::Time::now().toSec());
    if (!std::isfinite(reference_start) || reference_start<0) throw std::runtime_error("invalid reference time origin");
    if (!optimizer.OptimizeTrajectory_lbfgs(head,tail,positions.block(0,1,3,guess.getPieceNum()-1),guess.getDurations(),control,formation,reference_start))
      throw std::runtime_error("native optimizer did not produce an accepted candidate");
    auto trajectory=optimizer.getMinJerkOptPtr()->getTraj();
    for (double t=0.;t<=trajectory.getTotalDuration();t+=.01)
      if (!map->isInMap(trajectory.getPos(t)) || map->getInflateOccupancy(trajectory.getPos(t))==1)
        throw std::runtime_error("nominal trajectory fails full-interval map check");
    if (!nh.getParam("",after) || before!=after) throw std::runtime_error("planner parameters changed during query");
    result.put("status","FEASIBLE");result.put("reason","NATIVE_SWARM_STATIC_MAP_QUERY");
    result.put("scope","nominal reference only; qn tracking and full task validity not certified");
    result.put("duration_s",trajectory.getTotalDuration());result.put("max_speed_mps",trajectory.getMaxVelRate());
    result.put("reference_start_time",reference_start);
    result.put("formation",formation);result.put("formation_nodes",formation_size);result.put("input_point_count",points.size());
    ptree durations,coefficients;
    for (const auto &piece:trajectory) {
      ptree d;d.put_value(piece.getDuration());durations.push_back({"",d});
      ptree matrix;
      for (int row=0;row<3;++row) matrix.push_back({"",values(piece.getCoeffMat().row(row).transpose())});
      coefficients.push_back({"",matrix});
    }
    result.add_child("durations",durations);result.add_child("coefficients",coefficients);
  } catch (const std::exception &error) {result.put("reason",error.what());}
  boost::property_tree::write_json(output,result);
  return result.get<std::string>("status")=="FEASIBLE"?0:1;
}
