#include "simgrid/Exception.hpp"
#include "simgrid/plugins/energy.h"
#include "simgrid/s4u/Engine.hpp"
#include "src/surf/network_interface.hpp"
#include "src/surf/surf_interface.hpp"
#include "surf/surf.hpp"
#include "simgrid/s4u/Host.hpp"
#include "simgrid/s4u/Link.hpp"
#include "src/surf/network_wifi.hpp"


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

            void update(simgrid::kernel::resource::NetworkAction const&);
            inline double getConsumedEnergy(void){return eTot_;}

        private:
            s4u::Link* link_{};

            double eTot_    {0.0};
            double prev_update_ {0.0};

            // Same values as ns3:
            // https://www.nsnam.org/docs/release/3.30/doxygen/classns3_1_1_wifi_radio_energy_model.html#details
            double pIdle_  {0.82};
            double pTx_    {1.14};
            double pRx_    {0.94};
            double pSleep_ {0.10};

        };

        xbt::Extension<s4u::Link, LinkEnergyWifi> LinkEnergyWifi::EXTENSION_ID;

        void LinkEnergyWifi::update(simgrid::kernel::resource::NetworkAction const& action)
        {
          XBT_DEBUG("state updated for link %s, usage: %f, lat:%f, state:%d, started: %f finished: %f",
                  link_->get_cname(),link_->get_usage(),link_->get_latency(),
                  action.get_state(),action.get_start_time(),action.get_finish_time());
          double duration = surf_get_clock()-prev_update_;
          prev_update_ = surf_get_clock();

          // what is the cost?
          // 1Tx 1Rx from the AP
          // 1Tx from sender
          // 1Rx for receiver
          // ??
          eTot_ += (2*pTx_ + 2*pRx_)*duration*(link_->get_usage()/link_->get_bandwidth()); // just as an illustration, not true at all
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
        link.extension_set(new LinkEnergyWifi(&link));
      }else{
        XBT_DEBUG("Not Wifi Link: %s", link.get_cname());
      }
  });


  simgrid::s4u::Link::on_destruction.connect([](simgrid::s4u::Link const& link){
      if(link.get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI){
        XBT_DEBUG("Link %s destroyed, consumed: %f J", link.get_cname(), link.extension<LinkEnergyWifi>()->getConsumedEnergy());
      }
  });

  simgrid::s4u::Link::on_communication_state_change.connect([](simgrid::kernel::resource::NetworkAction const& action,
                                                               simgrid::kernel::resource::Action::State previous){
      XBT_DEBUG("New action");
      for (simgrid::kernel::resource::LinkImpl* link : action.links()) {
        if (link != nullptr && link->get_sharing_policy() == simgrid::s4u::Link::SharingPolicy::WIFI ){
          link->get_iface()->extension<LinkEnergyWifi>()->update(action);
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
          link->get_iface()->extension<LinkEnergyWifi>()->update(action);
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
