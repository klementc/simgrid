#include "simgrid/Exception.hpp"
#include "simgrid/plugins/energy.h"
#include "simgrid/s4u/Engine.hpp"
#include "src/surf/network_interface.hpp"
#include "src/surf/surf_interface.hpp"
#include "surf/surf.hpp"
#include "simgrid/s4u/Host.hpp"
#include "simgrid/s4u/Link.hpp"
#include "src/surf/network_wifi.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>


SIMGRID_REGISTER_PLUGIN(link_energy_wifi, "Energy wifi test", &sg_link_wifi_plugin_init);

XBT_LOG_NEW_DEFAULT_SUBCATEGORY(link_energy_wifi, surf, "Logging specific to the link energy wifi plugin");


namespace simgrid {
    namespace plugin {

        class LinkEnergyWifi {

        public:
            static simgrid::xbt::Extension<simgrid::s4u::Link, LinkEnergyWifi> EXTENSION_ID;

            explicit LinkEnergyWifi(simgrid::s4u::Link* ptr)
                    : link_(ptr)
            {
            }

            ~LinkEnergyWifi() = default;
            LinkEnergyWifi() = delete;

            void update(/*simgrid::kernel::resource::NetworkAction const&*/);
            inline double getConsumedEnergy(void){return eTot_;}
	  inline double getEdyn(void) {return eDyn_;}
	  inline double getEstat(void) {return eStat_;}
	          void setpIdle(double value) { pIdle_=value;}
	          void setpTx(double value) {pTx_=value;}
  	        void setpRx(double value) {pRx_=value;}
	          void setpSleep(double value) {pSleep_=value;}
            void init_watts_range_list();

        private:
            s4u::Link* link_{};
	  double eStat_   {0.0};
	  double eDyn_    {0.0};
            double eTot_    {0.0};
            double prev_update_ {0.0};

            // Same values as ns3:
            // https://www.nsnam.org/docs/release/3.30/doxygen/classns3_1_1_wifi_radio_energy_model.html#details
        	  double pIdle_  {0.82};
            double pTx_    {1.14};
            double pRx_    {0.94};
            double pSleep_ {0.10};

            bool valuesInit_ {false};
        };

        xbt::Extension<s4u::Link, LinkEnergyWifi> LinkEnergyWifi::EXTENSION_ID;

        void LinkEnergyWifi::update(/*simgrid::kernel::resource::NetworkAction const& action*/)
        {
          init_watts_range_list();
          XBT_DEBUG("state updated for link %s, usage: %f, lat:%f"/*, state:%d, started: %f finished: %f"*/,
                  link_->get_cname(),link_->get_usage(),link_->get_latency()
                  /*action.get_state(),action.get_start_time(),action.get_finish_time()*/);

          //init_watts_range_list();

          double duration = surf_ge_tclock()-prev_update_;
          prev_update_ = surf_get_clock();

          simgrid::kernel::resource::NetworkWifiLink* wifi_link = static_cast<simgrid::kernel::resource::NetworkWifiLink*>(link_->get_impl());
          wifi_link->get_nb_hosts_on_link();
          // what is the cost?
          // 1Tx 1Rx from the AP
          // 1Tx from sender
          // 1Rx for receiver
          // ??
	  eDyn_ += pIdle_*duration*(1 + wifi_link->get_nb_hosts_on_link());
	  eStat_ += ((pTx_-pIdle_) + (pRx_-pIdle_)) * duration *
	    (link_->get_usage()/link_->get_bandwidth());
          eTot_ += pIdle_*duration*(1 + wifi_link->get_nb_hosts_on_link()) +
	    ((pTx_-pIdle_) + (pRx_-pIdle_)) * duration *
	    (link_->get_usage()/link_->get_bandwidth()); // just as an illustration
        }
        
        void LinkEnergyWifi::init_watts_range_list()
        {
          if(valuesInit_)
            return;
          valuesInit_=true;
          const char* all_power_values_str = this->link_->get_property("wifi_watt_values");
          if (all_power_values_str == nullptr)
            return;

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

            XBT_DEBUG("Values initialized with: pSleep=%f pIdle=%f pTx=%f pRx=%f", pSleep_, pIdle_, pTx_, pRx_);
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
  simgrid::s4u::Link::on_creation.connect([](simgrid::s4u::Link& link){
      if(link.get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI){
        XBT_DEBUG("Wifi Link created: %s", link.get_cname());
	      LinkEnergyWifi* plugin = new LinkEnergyWifi(&link);
        link.extension_set(plugin);
      }else{
        XBT_DEBUG("Not Wifi Link: %s", link.get_cname());
      }
  });


  simgrid::s4u::Link::on_destruction.connect([](simgrid::s4u::Link const& link){
      if(link.get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI){
        link.extension<LinkEnergyWifi>()->update(/*action*/);
        XBT_INFO("Link %s destroyed, consumed: %f J dyn: %f stat: %f", link.get_cname(), link.extension<LinkEnergyWifi>()->getConsumedEnergy());
      }
  });

  simgrid::s4u::Link::on_communication_state_change.connect([](simgrid::kernel::resource::NetworkAction const& action,
                                                               simgrid::kernel::resource::Action::State previous){
      XBT_DEBUG("New action");
      for (simgrid::kernel::resource::LinkImpl* link : action.links()) {
        if (link != nullptr && link->get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI ){
          link->get_iface()->extension<LinkEnergyWifi>()->update(/*action*/);
        }
      }
  });

  simgrid::s4u::Link::on_communicate.connect([](const simgrid::kernel::resource::NetworkAction& action,
                                                const simgrid::s4u::Host* host1,
                                                const simgrid::s4u::Host* host2){
      XBT_DEBUG("communication started from %s to %s", host1->get_cname(), host2->get_cname());
      for (simgrid::kernel::resource::LinkImpl* link : action.links()) {

        if ( link->get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI ){
          simgrid::kernel::resource::NetworkWifiLink* wifi_link = static_cast<simgrid::kernel::resource::NetworkWifiLink*>(link);
          XBT_DEBUG("going by wifi load:%f bandwidth:%f rate1:%f rate2:%f", wifi_link->get_load(),
                  wifi_link->get_bandwidth(),wifi_link->get_host_rate(host1), wifi_link->get_host_rate(host2));
          link->get_iface()->extension<LinkEnergyWifi>()->update(/*action*/);
        }else{
          XBT_DEBUG("going by non wifi %s usage:%f started:%lf state:%d BW:%f latency:%f", link->get_cname(),
                   link->get_iface()->get_usage(), action.get_start_time(), action.get_state(),
                   link->get_bandwidth(), link->get_latency());
        }
      }
      XBT_DEBUG("--------------");
  });

  simgrid::s4u::Engine::on_simulation_end.connect([](){XBT_INFO("Simulation finished");});

}
