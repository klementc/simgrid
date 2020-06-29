/* Copyright (c) 2017-2020. The SimGrid Team. All rights reserved.          */

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the license (GNU LGPL) which comes with this package. */

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
/** @degroup plugin_link_energy_wifi Plugin WiFi energy
 * 
 * This is the WiFi energy plugin, accounting for the dissipated energy of WiFi links.
 */

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
   * Update the energy consumed by link_ when transmissions start or end
   */
  void update(const simgrid::kernel::resource::NetworkAction &);
  
  /**
   * Update the energy consumed when link_ is destroyed
   */
  void updateDestroy();

  /**
   * Fetches energy consumption values from the platform file.
   * The user can specify:
   *  - wifi_watt_values: energy consumption in each state (IDLE:Tx:Rx:SLEEP)
   *      default: 0.82:1.14:0.94:0.10
   *  - controlDuration: duration of active beacon transmissions per second
   *      default: 0.0036
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
  
  
private:
  // associative array keeping what has already been sent for a given action (required for interleaved actions)
  std::map<simgrid::kernel::resource::NetworkWifiAction *, std::pair<int, double>> flowTmp{};

  // WiFi link the plugin instance is attached to
  s4u::Link* link_{};
  
  // dynamic energy (active durations consumption)
  double eDyn_{0.0};
  // static energy (no activity consumption)
  double eStat_{0.0};

  // duration since previous energy update
  double prev_update_{0.0};

  // Same energy calibration values as ns3 by default
  // https://www.nsnam.org/docs/release/3.30/doxygen/classns3_1_1_wifi_radio_energy_model.html#details
  double pIdle_{0.82};
  double pTx_{1.14};
  double pRx_{0.94};
  double pSleep_{0.10};

  // constant taking beacons into account
  double controlDuration_{0.0036};

  // Measurements for report
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

  // add IDLE energy usage, as well as beacons cost
  eDyn_+=duration*controlDuration_*wifi_link->get_nb_hosts_on_link()*pRx_;
  eStat_ += (duration-(duration*controlDuration_)) * pIdle_ * (wifi_link->get_nb_hosts_on_link()+1);

  XBT_DEBUG("finish eStat_ += %f * %f * (%d+1) | eStat = %f", duration, pIdle_, wifi_link->get_nb_hosts_on_link(), eStat_);
}

void LinkEnergyWifi::update(const simgrid::kernel::resource::NetworkAction& action) {
  bool finishedAFlow = false;
  init_watts_range_list();

  double duration = surf_get_clock() - prev_update_;
  prev_update_    = surf_get_clock();

  if(duration < 1e-6) {
    XBT_DEBUG("duration equals to 0, leaving update");
    return;
  }

  simgrid::kernel::resource::NetworkWifiLink* wifi_link =
      static_cast<simgrid::kernel::resource::NetworkWifiLink*>(link_->get_impl());
  
  //sum_{actions du lien} action->get_variable()/wifi_link->get_host_rate(dst)
  const kernel::lmm::Variable* var;
  const kernel::lmm::Element* elem = nullptr;

  double durUsage = 0;
  while((var = wifi_link->get_constraint()->get_variable(&elem))) {
    auto* action = static_cast<kernel::resource::NetworkWifiAction*>(var->get_id());
    XBT_DEBUG("cost: %f action value: %f link rate 1: %f link rate 2: %f", action->get_cost(), action->get_variable()->get_value(), wifi_link->get_host_rate(&action->get_src()),wifi_link->get_host_rate(&action->get_dst()));
    action->get_variable();
    
    double du = 0;
    std::map<simgrid::kernel::resource::NetworkWifiAction *, std::pair<int, double>>::iterator it;

    if(action->get_variable()->get_value()) {
      it = flowTmp.find(action);
      
      if(it == flowTmp.end()){
        flowTmp[action] = std::pair<int,double>(0, action->get_start_time());
      }

      it = flowTmp.find(action);
      int lsize = it->second.first;
      double ltime = it->second.second;

      du = (action->get_cost()-it->second.first) / action->get_variable()->get_value();

      XBT_DEBUG("action from array: %d %f durU: %f (cost:%f value: %f) acfst: %d acsnd:%f", lsize, ltime, du,  action->get_cost(), action->get_variable()->get_value(), it->second.first, it->second.second );

      if(du > surf_get_clock()-it->second.second) {
        XBT_DEBUG("%f -> %f", du, surf_get_clock()-it->second.second);
        du = surf_get_clock()-it->second.second;
      }
      XBT_DEBUG("Durusage: %f du: %f", durUsage, du);
      if(du > durUsage) {
        XBT_DEBUG("DurUsage new value!: %f", du);
        durUsage = du;
      }
      if(action->get_finish_time() ==  surf_get_clock()){
        finishedAFlow = true;
      }

      it->second.first += du*action->get_variable()->get_value();
      it->second.second =  surf_get_clock();

      if(it->second.first >= action->get_cost()){
        flowTmp.erase (it);
      }
      /*
      if(! dans table)
        mettre dedans
      
      du = action.get_cost()-table.size / action.get_variable()->get_value()
      si du > now-table.last
        du = now-last
      si du > durUsage
        durusage=du

      mettre a jour:
        table.size += du*debit
        table.last = now
        */
    }
  }

  if(kernel::resource::NetworkModel::cfg_crosstraffic) {
    XBT_DEBUG("Cross traffic activated, divide dirUsage by 2 %f -> %f", durUsage, durUsage/2);
    durUsage/=2;
  }
  XBT_DEBUG("durUsage: %f", durUsage);

  // control cost
  eDyn_+=duration*controlDuration_*wifi_link->get_nb_hosts_on_link()*pRx_;
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
    eStat_ += (duration-(duration*controlDuration_)) * pIdle_ * (wifi_link->get_nb_hosts_on_link()+1);
  }

  XBT_DEBUG("eStat_ += %f * %f * (%d+1) | eStat = %f", duration, pIdle_, wifi_link->get_nb_hosts_on_link(), eStat_);
}
  


void LinkEnergyWifi::init_watts_range_list()
{
  if (valuesInit_)
    return;
  valuesInit_                      = true;

  /* beacons factor
  Set to 0 if you do not want to compute beacons,
  otherwise to the duration of beacons transmissions per second
  */
  const char* beacons_factor = this->link_->get_property("controlDuration");
  if(beacons_factor != nullptr) {
    try {
      controlDuration_ = std::stod(beacons_factor);
    } catch (const std::invalid_argument&) {
      throw std::invalid_argument(std::string("Invalid beacons factor value for link ") + this->link_->get_cname());
    }
  }

  const char* all_power_values_str = this->link_->get_property("wifi_watt_values");

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
