#include "simgrid/Exception.hpp"
#include "simgrid/plugins/energy.h"
#include "simgrid/s4u/Engine.hpp"
#include "simgrid/s4u/Host.hpp"
#include "simgrid/s4u/Link.hpp"
#include "src/surf/network_interface.hpp"
#include "src/surf/network_wifi.hpp"
#include "src/surf/surf_interface.hpp"
#include "surf/surf.hpp"
#include "src/kernel/lmm/maxmin.hpp"
#include "xbt/config.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

SIMGRID_REGISTER_PLUGIN(link_energy_wifi, "Energy wifi test", &sg_link_wifi_plugin_init);

XBT_LOG_NEW_DEFAULT_SUBCATEGORY(link_energy_wifi, surf, "Logging specific to the link energy wifi plugin");

namespace simgrid {
namespace plugin {

class LinkEnergyWifi {

public:
  static simgrid::xbt::Extension<simgrid::s4u::Link, LinkEnergyWifi> EXTENSION_ID;

  explicit LinkEnergyWifi(simgrid::s4u::Link* ptr) : link_(ptr) {}

  ~LinkEnergyWifi() = default;
  LinkEnergyWifi()  = delete;

  /**
   * Update the energy consumed by the link
   * (called when a communication happens)
   */
  void update(const simgrid::kernel::resource::NetworkAction &);
  
  /**
   * Update the idle energy when the link is destroyed
   * (required to count the energy consumed between the last communication and the link's destruction)
   */
  void updateDestroy();

  /**
   * Fetches energy consumption values from the platform file
   */
  void init_watts_range_list();

  /* Getters */
  double getConsumedEnergy(void) { return eDyn_+eStat_; }
  double getEdyn(void) {return eDyn_; }
  double getEstat(void) {return eStat_; }
  double getDurTxRx(void){return durTxRx;}
  double getDurIdle(void){return durIdle;}
  
  /* Setters */
  void setpIdle(double value) { pIdle_ = value; }
  void setpTx(double value) { pTx_ = value; }
  void setpRx(double value) { pRx_ = value; }
  void setpSleep(double value) { pSleep_ = value; }
  void setBeacons(bool val){beacons=val;}
  
  
private:
  s4u::Link* link_{};

  // boolean to know if we compute the energy of beacons or not
  bool beacons{true};
  
  // dynamic energy (ns3 comparison)
  double eDyn_{0.0};
  
  // static energy (ns3 comparison)
  double eStat_{0.0};

  // duration since last energy update
  double prev_update_{0.0};

  // Same energy calibration values as ns3 by default
  // https://www.nsnam.org/docs/release/3.30/doxygen/classns3_1_1_wifi_radio_energy_model.html#details
  double pIdle_{0.82};
  double pTx_{1.14};
  double pRx_{0.94};
  double pSleep_{0.10};

  // constant taking beacons into account
  const double controlDuration_{0.00186754873};

  // DIY crap
  double durTxRx{0};
  double durIdle{0};
  bool valuesInit_{false};
};

xbt::Extension<s4u::Link, LinkEnergyWifi> LinkEnergyWifi::EXTENSION_ID;

void LinkEnergyWifi::updateDestroy() {
  simgrid::kernel::resource::NetworkWifiLink* wifi_link =
    static_cast<simgrid::kernel::resource::NetworkWifiLink*>(link_->get_impl());
  double duration = surf_get_clock() - prev_update_;
  prev_update_    = surf_get_clock();

  durIdle+=duration;
  eStat_ += duration * pIdle_ * (wifi_link->get_nb_hosts_on_link()+1);

  // control cost
  if(beacons) {
    eDyn_+=duration*controlDuration_*wifi_link->get_nb_hosts_on_link()*pRx_;
  }
  
  XBT_DEBUG("finish eStat_ += %f * %f * (%d+1) | eStat = %f", duration, pIdle_, wifi_link->get_nb_hosts_on_link(), eStat_);
}

void LinkEnergyWifi::update(const simgrid::kernel::resource::NetworkAction& action) {

  init_watts_range_list();
  //XBT_DEBUG("state updated for link %s, usage: %f, lat:%f" /*, state:%d, started: %f finished: %f"*/,
  //          link_->get_cname(), link_->get_usage(), link_->get_latency()
  //         /*action.get_state(),action.get_start_time(),action.get_finish_time()*/);

  double duration = surf_get_clock() - prev_update_;
  prev_update_    = surf_get_clock();

  // todo epsilon
  if(duration < 1e-6) {
    XBT_DEBUG("duration equals to 0, leaving update");
    return;
  }
  //XBT_INFO("duration different than 0, continue update");

  simgrid::kernel::resource::NetworkWifiLink* wifi_link =
      static_cast<simgrid::kernel::resource::NetworkWifiLink*>(link_->get_impl());
  
  //sum_{actions du lien} action->get_variable()/wifi_link->get_host_rate(dst)
  //const kernel::lmm::Variable* var;
  //const kernel::lmm::Element* elem = nullptr;
  double durUsage = 0;
  
  //const simgrid::kernel::resource::NetworkWifiAction& actionWifi = dynamic_cast<const simgrid::kernel::resource::NetworkWifiAction&>(action);
  //if (actionWifi.get_src_link() == link_->get_impl())
  if(action.get_variable()->get_value()!=0)
      durUsage += action.get_cost() / action.get_variable()->get_value();
  //else if (actionWifi.get_dst_link() == link_->get_impl())
  //    durUsage += action.get_cost() / action.get_variable()->get_value();
  /*while((var = wifi_link->get_constraint()->get_variable(&elem))) {
    auto* action = static_cast<kernel::resource::NetworkWifiAction*>(var->get_id());
    XBT_DEBUG("cost: %f action value: %f link rate 1: %f link rate 2: %f", action->get_cost(), action->get_variable()->get_value(), wifi_link->get_host_rate(&action->get_src()),wifi_link->get_host_rate(&action->get_dst()));
    action->get_variable();
    if (actionWifi.get_src_link() == link_->get_impl())
      durUsage += action->get_cost() / action->get_variable()->get_value();
      //durUsage += action->get_variable()->get_value()/wifi_link->get_host_rate(&action->get_src());
    else if (actionWifi.get_dst_link() == link_->get_impl())
      durUsage += action->get_cost() / action->get_variable()->get_value();
      //durUsage += action->get_variable()->get_value()/wifi_link->get_host_rate(&action->get_dst());
    else{
      xbt_die("update an invalide link (update energy)");
    }
  }*/

  if(kernel::resource::NetworkModel::cfg_crosstraffic) {
    XBT_DEBUG("Cross traffic activated, divide dirUsage by 2 %f -> %f", durUsage, durUsage/2);
    durUsage/=2;
  }
  XBT_DEBUG("durUsage: %f", durUsage);
  /*
    while ((var = get_constraint()->get_variable(&elem))) {
    auto* action = static_cast<CpuCas01Action*>(var->get_id());

    get_model()->get_maxmin_system()->update_variable_bound(action->get_variable(),
                                                            action->requested_core() * speed_.scale * speed_.peak);
  }
  */

/*  double durUsage = 0;

  if (actionWifi.get_src_link() == link_->get_impl())
    durUsage = action.get_variable()->get_value()/wifi_link->get_host_rate(&action.get_src());
  else if (actionWifi.get_dst_link() == link_->get_impl())
    durUsage = action.get_variable()->get_value()/wifi_link->get_host_rate(&action.get_dst());
  else{
    xbt_die("update an invalide link (update energy)");
  }*/

  // control cost
  if(beacons) {
    eDyn_+=duration*controlDuration_*wifi_link->get_nb_hosts_on_link()*pRx_;
  }
  /**
   * As in ns3:
   *  - if tx or rx (dyn consumption) i.e. get_usage > 0 -> Pdyn+=duration*(get_nb_hosts_on_link*pRx + 1*pTx)
   *  - if idle i.e. get_usage = 0 -> Pstat+=pIdle*get_nb_hosts_on_link*durarion
   * Ptot = Pdyn+Pstat
   */
  if(link_->get_usage()){
    eDyn_ += /*duration * */durUsage * ((wifi_link->get_nb_hosts_on_link()*pRx_)+pTx_);
    eStat_ += (duration-durUsage)* pIdle_ * (wifi_link->get_nb_hosts_on_link()+1);
    XBT_DEBUG("eDyn +=  %f * ((%d * %f) + %f) | eDyn = %f (durusage =%f)", durUsage, wifi_link->get_nb_hosts_on_link(), pRx_, pTx_, eDyn_, durUsage);
    durTxRx+=duration;
  }else{
    durIdle+=duration;
    eStat_ += duration * pIdle_ * (wifi_link->get_nb_hosts_on_link()+1);
    XBT_DEBUG("eStat_ += %f * %f * (%d+1) | eStat = %f", duration, pIdle_, wifi_link->get_nb_hosts_on_link(), eStat_);
  }
  
}

void LinkEnergyWifi::init_watts_range_list()
{
  if (valuesInit_)
    return;
  valuesInit_                      = true;
  const char* all_power_values_str = this->link_->get_property("wifi_watt_values");
  const char* compute_beacons = this->link_->get_property("beacons");

  if(compute_beacons != nullptr && strcmp(compute_beacons, "false")==0) {
    setBeacons(false);
    XBT_INFO("Not computing the energy consumption of beacons");
  }else {
    XBT_INFO("Computing the energy consumption of beacons");
  }

  if (all_power_values_str != nullptr)
  {
    std::vector<std::string> all_power_values;
    boost::split(all_power_values, all_power_values_str, boost::is_any_of(","));

    for (auto current_power_values_str : all_power_values) {
      /* retrieve the power values associated */
      std::vector<std::string> current_power_values;
      boost::split(current_power_values, current_power_values_str, boost::is_any_of(":"));
      xbt_assert(current_power_values.size() == 4,
                "Power properties incorrectly defined - could not retrieve idle, Tx, Rx, Sleep power values for link %s",
                this->link_->get_cname());

      /* min_power corresponds to the idle power (link load = 0) */
      /* max_power is the power consumed at 100% link load       */
      try {
        pSleep_ = std::stod(current_power_values.at(3));
      } catch (const std::invalid_argument&) {
        throw std::invalid_argument(std::string("Invalid idle power value for link ") + this->link_->get_cname());
      }
      try {
        pRx_ = std::stod(current_power_values.at(2));
      } catch (const std::invalid_argument&) {
        throw std::invalid_argument(std::string("Invalid idle power value for link ") + this->link_->get_cname());
      }
      try {
        pTx_ = std::stod(current_power_values.at(1));
      } catch (const std::invalid_argument&) {
        throw std::invalid_argument(std::string("Invalid idle power value for link ") + this->link_->get_cname());
      }
      try {
        pIdle_ = std::stod(current_power_values.at(0));
      } catch (const std::invalid_argument&) {
        throw std::invalid_argument(std::string("Invalid busy power value for link ") + this->link_->get_cname());
      }

      XBT_DEBUG("Values aa initialized with: pSleep=%f pIdle=%f pTx=%f pRx=%f", pSleep_, pIdle_, pTx_, pRx_);
    }
  }
}

} // namespace plugin
} // namespace simgrid

using simgrid::plugin::LinkEnergyWifi;

void sg_link_wifi_plugin_init()
{
  XBT_DEBUG("Init wifi energy plugin");
  if (LinkEnergyWifi::EXTENSION_ID.valid())
    return;
  LinkEnergyWifi::EXTENSION_ID = simgrid::s4u::Link::extension_create<LinkEnergyWifi>();

  // attach to events
  simgrid::s4u::Link::on_creation.connect([](simgrid::s4u::Link& link) {
    if (link.get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI) {
      XBT_DEBUG("Wifi Link created: %s", link.get_cname());
      LinkEnergyWifi* plugin = new LinkEnergyWifi(&link);
      link.extension_set(plugin);
    } else {
      XBT_DEBUG("Not Wifi Link: %s", link.get_cname());
    }
  });

  simgrid::s4u::Link::on_destruction.connect([](simgrid::s4u::Link const& link) {
    if (link.get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI) {
      /*
        compter l'energie entre fin de comm et fin de l'expe
      */
      //link.extension<LinkEnergyWifi>()->update(action);
      link.extension<LinkEnergyWifi>()->updateDestroy();
      XBT_INFO("Link %s destroyed, consumed: %f J dyn: %f stat: %f durIdle: %f durTxRx: %f", link.get_cname(),
               link.extension<LinkEnergyWifi>()->getConsumedEnergy(),
               link.extension<LinkEnergyWifi>()->getEdyn(),
               link.extension<LinkEnergyWifi>()->getEstat(),
	       link.extension<LinkEnergyWifi>()->getDurIdle(),
	       link.extension<LinkEnergyWifi>()->getDurTxRx()
	       );
    }
  });

  simgrid::s4u::Link::on_communication_state_change.connect(
      [](simgrid::kernel::resource::NetworkAction const& action, simgrid::kernel::resource::Action::State previous) {
        //XBT_DEBUG("New action");
        for (simgrid::kernel::resource::LinkImpl* link : action.get_links()) {
          if (link != nullptr && link->get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI) {
            link->get_iface()->extension<LinkEnergyWifi>()->update(action);
          }
        }
      });

  simgrid::s4u::Link::on_communicate.connect([](const simgrid::kernel::resource::NetworkAction& action) {
    const simgrid::kernel::resource::NetworkWifiAction* actionWifi = dynamic_cast<const simgrid::kernel::resource::NetworkWifiAction*>(&action);

    if(actionWifi == nullptr) 
      return;

    //XBT_DEBUG("communication started from %s to %s", action->get_source(), host2->get_dst());
    auto link_src  = actionWifi->get_src_link();
    auto link_dst = actionWifi->get_dst_link();

    if(link_src != nullptr)
      link_src->get_iface()->extension<LinkEnergyWifi>()->update(action);
    if(link_dst != nullptr)
      link_dst->get_iface()->extension<LinkEnergyWifi>()->update(action);

  });

  simgrid::s4u::Engine::on_simulation_end.connect([]() { XBT_INFO("Simulation finished"); });
}
