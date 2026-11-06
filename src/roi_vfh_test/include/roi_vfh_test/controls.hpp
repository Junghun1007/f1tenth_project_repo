#pragma once
#include "roi_vfh_test/planner.hpp"
#include <opencv2/imgproc.hpp>
#include <array>
#include <iomanip>
#include <sstream>

namespace roi_vfh_test {
struct PlannerField {
  const char* name;
  double Config::* member;
  double minimum,maximum;
  const char* unit;
};
inline const std::array<PlannerField,11>& plannerFields() {
  static const std::array<PlannerField,11> fields{{
    {"nominal_speed_mps",&Config::speed,.05,1,"max speed, m/s (0.05..1)"},
    {"lane_lookahead_m",&Config::lane_lookahead,.15,.8,"lane tracking, m (0.15..0.8)"},
    {"safety_margin_m",&Config::margin,0,.2,"each side / front, m (0..0.2)"},
    {"horizon_m",&Config::horizon,.3,2.5,"planning length, m (0.3..2.5)"},
    {"reference_distance_m",&Config::reference_distance,.3,3,"VFH target, m (0.3..3)"},
    {"sample_step_m",&Config::step,.02,.08,"path sample, m (0.02..0.08)"},
    {"lane_departure_m",&Config::departure,0,.4,"body departure, m (0..0.4)"},
    {"deceleration_mps2",&Config::deceleration,.1,3,"m/s^2; startup ceiling applies"},
    {"response_delay_sec",&Config::delay,0,1,"response delay, sec (0..1)"},
    {"histogram_low",&Config::hist_low,0,999,"VFH low (must be < high)"},
    {"histogram_high",&Config::hist_high,.001,1000,"VFH high (must be > low)"}
  }};
  return fields;
}
inline const PlannerField* plannerField(const std::string& name) {
  for(const auto& field:plannerFields())if(name==field.name)return &field;
  return nullptr;
}
inline void validatePlannerEdits(const Config& config,double deceleration_ceiling) {
  for(const auto& f:plannerFields()) {
    const double value=config.*(f.member);
    if(!std::isfinite(value) || value<f.minimum || value>f.maximum)
      throw std::invalid_argument(std::string(f.name)+": outside editable limits");
  }
  if(config.hist_low>=config.hist_high)throw std::invalid_argument("histogram_low must be smaller than histogram_high");
  if(config.deceleration>deceleration_ceiling+1e-9)
    throw std::invalid_argument("deceleration_mps2 exceeds startup controller ceiling");
  config.validate();
}
class PlannerControls {
  struct Edit {double observed{};std::string draft;bool pending{false};};
  std::array<Edit,11> edits_;
  bool ready_{false},open_{false},select_all_{false};
  int active_{-1};
  std::string message_{"Edit values, then Apply. Start also applies valid edits."};
  static std::string format(double value){std::ostringstream s;s<<std::setprecision(8)<<value;return s.str();}
  static cv::Rect field(std::size_t i){return {250+int(i/6)*374,166+int(i%6)*48,104,30};}
  static cv::Rect applyButton(){return {16,488,350,36};}
  static cv::Rect reloadButton(){return {390,488,350,36};}
public:
  enum class Action {None,Apply,Reload};
  static cv::Rect toggleButton(){return {366,12,170,36};}
  bool editing()const{return active_>=0;}
  void blur(){active_=-1;}
  void show(bool value){open_=value;active_=-1;}
  void toggle(){show(!open_);}
  void message(const std::string& text){message_=text;}
  void sync(const Config& config) {
    for(std::size_t i=0;i<edits_.size();++i) {
      auto& e=edits_[i];const double value=config.*(plannerFields()[i].member);
      if(!ready_ || value!=e.observed){e.observed=value;e.draft=format(value);e.pending=false;}
    }
    ready_=true;
  }
  void reload(const Config& config){ready_=false;active_=-1;sync(config);}
  std::vector<std::pair<std::string,double>> changes()const {
    std::vector<std::pair<std::string,double>> result;
    for(std::size_t i=0;i<edits_.size();++i) {
      const auto& e=edits_[i];if(!e.pending)continue;
      std::size_t used=0;double value=0;
      try {value=std::stod(e.draft,&used);}catch(const std::exception&){throw std::invalid_argument(std::string(plannerFields()[i].name)+": enter a number");}
      if(used!=e.draft.size() || !std::isfinite(value))throw std::invalid_argument(std::string(plannerFields()[i].name)+": enter a finite number");
      if(value!=e.observed)result.emplace_back(plannerFields()[i].name,value);
    }
    return result;
  }
  Action click(int x,int y,bool locked) {
    if(!open_ || locked)return Action::None;
    for(std::size_t i=0;i<edits_.size();++i)if(field(i).contains(cv::Point(x,y))){active_=int(i);select_all_=true;return Action::None;}
    active_=-1;
    if(applyButton().contains(cv::Point(x,y)))return Action::Apply;
    if(reloadButton().contains(cv::Point(x,y)))return Action::Reload;
    return Action::None;
  }
  Action key(int key,bool locked) {
    if(!open_ || locked){active_=-1;return Action::None;}
    if(key==13 || key==10 || key=='a' || key=='A'){active_=-1;return Action::Apply;}
    if(key==9){active_=(active_+1)%int(edits_.size());select_all_=true;return Action::None;}
    if(active_<0)return Action::None;
    auto& e=edits_[active_];
    if(key==27){e.draft=format(e.observed);e.pending=false;active_=-1;return Action::None;}
    if(key==8 || key==127 || key==0xff08) {
      if(select_all_)e.draft.clear();else if(!e.draft.empty())e.draft.pop_back();
    }else if((key>='0' && key<='9') || key=='.' || key=='-' || key=='+' || key=='e' || key=='E') {
      if(select_all_)e.draft.clear();if(e.draft.size()<24)e.draft.push_back(static_cast<char>(key));
    }else return Action::None;
    select_all_=false;e.pending=true;return Action::None;
  }
  void draw(cv::Mat& canvas,bool locked,double deceleration_ceiling)const {
    const auto text=[&](const std::string& value,cv::Point at,double scale,cv::Scalar color){cv::putText(canvas,value,at,cv::FONT_HERSHEY_SIMPLEX,scale,color,1,cv::LINE_AA);};
    cv::rectangle(canvas,toggleButton(),{120,110,75},-1);
    text(open_?"Back to view":"VFH parameters",{378,36},.49,{255,255,255});
    if(!open_)return;
    cv::rectangle(canvas,{0,102,760,518},{245,245,245},-1);
    text(locked?"VFH PARAMETERS - LOCKED until Stop":"VFH PARAMETERS - editable before Start",{16,124},.55,{35,35,35});
    text("Click and type. Enter/A: Apply all edits. Tab: next. Esc: cancel field.",{16,146},.40,{60,60,60});
    for(std::size_t i=0;i<edits_.size();++i) {
      const auto& e=edits_[i];const auto& f=plannerFields()[i];const auto box=field(i);const int left=16+int(i/6)*374;
      text(f.name,{left,box.y+12},.37,{35,35,35});text(f.unit,{left,box.y+30},.28,{100,100,100});
      cv::rectangle(canvas,box,locked?cv::Scalar(218,218,218):(e.pending?cv::Scalar(220,245,255):cv::Scalar(255,255,255)),-1);
      cv::rectangle(canvas,box,active_==int(i)?cv::Scalar(220,120,30):cv::Scalar(150,150,150),active_==int(i)?2:1);
      const auto value=e.draft.size()>11?e.draft.substr(e.draft.size()-11):e.draft;
      text(value+(active_==int(i)?"_":""),box.tl()+cv::Point(5,21),.44,{25,25,25});
    }
    cv::rectangle(canvas,applyButton(),locked?cv::Scalar(160,160,160):cv::Scalar(75,150,70),-1);
    cv::rectangle(canvas,reloadButton(),{175,175,175},-1);
    text("Apply edits (Enter / A)",{70,512},.48,{255,255,255});text("Reload / discard edits",{440,512},.48,{35,35,35});
    text(message_.substr(0,104),{16,550},.39,{35,65,190});
    text(cv::format("Deceleration ceiling: %.2f m/s^2 (controller startup setting)",deceleration_ceiling),{16,576},.40,{60,60,60});
    text("Start applies edits and locks ROI + VFH. Stop unlocks both.",{16,601},.40,{60,60,60});
  }
};
}
