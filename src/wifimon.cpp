#include "smd_wifi_monitor/wifimon.hpp"
#include "smd_wifi_monitor/WifiStatus.h"

#include <pluginlib/class_list_macros.h>

#include <net/if.h>
#include <netlink/attr.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <linux/nl80211.h>

#include <fstream>

PLUGINLIB_DECLARE_CLASS( wifimon, WifiMon, wifimon::WifiMon, nodelet::Nodelet )

namespace wifimon
{
	const std::string WifiMon::mac2str( unsigned char *arg )
	{
		char tmp[21];
		int l = 0;
		for ( int i = 0; i < 6; i++ )
		{
			if (i == 0)
			{
				sprintf( tmp + l, "%02x", arg[i] );
				l += 2;
			}
			else
			{
				sprintf( tmp + l, ":%02x", arg[i] );
				l += 3;
			}
		}
		tmp[20] = 0;
		return tmp;
	}

	const char * WifiMon::iftype_name( enum nl80211_iftype iftype )
	{
		if( iftype <= NL80211_IFTYPE_MAX && iftype >= 0 && ifmodes[iftype] )
			return ifmodes[iftype];
		return ifmodes[(int)NL80211_IFTYPE_MAX + 1];
	}

	const char * WifiMon::bss_stat_name( enum nl80211_bss_status bss_stat )
	{
		if( bss_stat <= 3 && bss_stat > 0
			&& bss_stats[bss_stat] )
			return bss_stats[bss_stat];
		return bss_stats[4];
	}

	int WifiMon::get_link_sta_handler( struct nl_msg *msg, void *arg )
	{
		struct iface_info *info = (struct iface_info *)arg;
		struct genlmsghdr *gnlh = (struct genlmsghdr *)nlmsg_data(nlmsg_hdr(msg));
		struct nlattr *tb[NL80211_ATTR_MAX + 1];
		struct nlattr *sinfo[NL80211_STA_INFO_MAX + 1];
		struct nlattr *rinfo[NL80211_RATE_INFO_MAX + 1];
		struct nlattr *binfo[NL80211_STA_BSS_PARAM_MAX + 1];
		static struct nla_policy stats_policy[NL80211_STA_INFO_MAX + 1];
		stats_policy[NL80211_STA_INFO_INACTIVE_TIME].type = NLA_U32;
		stats_policy[NL80211_STA_INFO_RX_BYTES].type = NLA_U32;
		stats_policy[NL80211_STA_INFO_TX_BYTES].type = NLA_U32;
		stats_policy[NL80211_STA_INFO_RX_PACKETS].type = NLA_U32;
		stats_policy[NL80211_STA_INFO_TX_PACKETS].type = NLA_U32;
		stats_policy[NL80211_STA_INFO_SIGNAL].type = NLA_U8;
		stats_policy[NL80211_STA_INFO_TX_BITRATE].type = NLA_NESTED;
		stats_policy[NL80211_STA_INFO_LLID].type = NLA_U16;
		stats_policy[NL80211_STA_INFO_PLID].type = NLA_U16;
		stats_policy[NL80211_STA_INFO_PLINK_STATE].type = NLA_U8;
		static struct nla_policy rate_policy[NL80211_RATE_INFO_MAX + 1];
		rate_policy[NL80211_RATE_INFO_BITRATE].type = NLA_U16;
		rate_policy[NL80211_RATE_INFO_BITRATE32].type = NLA_U32;
		rate_policy[NL80211_RATE_INFO_MCS].type = NLA_U8;
		rate_policy[NL80211_RATE_INFO_40_MHZ_WIDTH].type = NLA_FLAG;
		rate_policy[NL80211_RATE_INFO_SHORT_GI].type = NLA_FLAG;
		static struct nla_policy bss_policy[NL80211_STA_BSS_PARAM_MAX + 1];
		bss_policy[NL80211_STA_BSS_PARAM_CTS_PROT].type = NLA_FLAG;
		bss_policy[NL80211_STA_BSS_PARAM_SHORT_PREAMBLE].type = NLA_FLAG;
		bss_policy[NL80211_STA_BSS_PARAM_SHORT_SLOT_TIME].type = NLA_FLAG;
		bss_policy[NL80211_STA_BSS_PARAM_DTIM_PERIOD].type = NLA_U8;
		bss_policy[NL80211_STA_BSS_PARAM_BEACON_INTERVAL].type = NLA_U16;

		nla_parse( tb, NL80211_ATTR_MAX, genlmsg_attrdata( gnlh, 0 ), genlmsg_attrlen( gnlh, 0 ), NULL );

		if( !tb[NL80211_ATTR_STA_INFO] || nla_parse_nested( sinfo, NL80211_STA_INFO_MAX,
			tb[NL80211_ATTR_STA_INFO], stats_policy ) )
		{
			info->valid = false;
			return NL_SKIP;
		}

		if( sinfo[NL80211_STA_INFO_RX_BYTES] )
			info->rx_bytes = nla_get_u32(sinfo[NL80211_STA_INFO_RX_BYTES]);
		if( sinfo[NL80211_STA_INFO_TX_BYTES] )
			info->tx_bytes = nla_get_u32(sinfo[NL80211_STA_INFO_TX_BYTES]);
		if( sinfo[NL80211_STA_INFO_RX_PACKETS] )
			info->rx_pkts = nla_get_u32(sinfo[NL80211_STA_INFO_RX_PACKETS]);
		if( sinfo[NL80211_STA_INFO_TX_PACKETS] )
			info->tx_pkts = nla_get_u32(sinfo[NL80211_STA_INFO_TX_PACKETS]);

		if( sinfo[NL80211_STA_INFO_SIGNAL] )
			info->signal = (int8_t)nla_get_u8( sinfo[NL80211_STA_INFO_SIGNAL] );

		if( sinfo[NL80211_STA_INFO_TX_BITRATE] )
		{
			if( !nla_parse_nested( rinfo, NL80211_RATE_INFO_MAX,
				sinfo[NL80211_STA_INFO_TX_BITRATE], rate_policy ) )
			{
				if( rinfo[NL80211_RATE_INFO_BITRATE32] )
					info->bitrate = nla_get_u32( rinfo[NL80211_RATE_INFO_BITRATE32] ) / 10.0;
				else if( rinfo[NL80211_RATE_INFO_BITRATE] )
					info->bitrate = nla_get_u16( rinfo[NL80211_RATE_INFO_BITRATE] ) / 10.0;
			}
		}

		return NL_SKIP;
	}

	int WifiMon::get_link_bss_handler( struct nl_msg *msg, void *arg )
	{
		struct iface_info *info = (struct iface_info *)arg;
		struct genlmsghdr *gnlh = (struct genlmsghdr *)nlmsg_data( nlmsg_hdr( msg ) );
		struct nlattr *tb[NL80211_ATTR_MAX + 1];
		struct nlattr *bss[NL80211_BSS_MAX + 1];
		static struct nla_policy bss_policy[NL80211_BSS_MAX + 1];
		bss_policy[NL80211_BSS_TSF].type = NLA_U64;
		bss_policy[NL80211_BSS_FREQUENCY].type = NLA_U32;
		bss_policy[NL80211_BSS_BEACON_INTERVAL].type = NLA_U16;
		bss_policy[NL80211_BSS_CAPABILITY].type = NLA_U16;
		bss_policy[NL80211_BSS_SIGNAL_MBM].type = NLA_U32;
		bss_policy[NL80211_BSS_SIGNAL_UNSPEC].type = NLA_U8;
		bss_policy[NL80211_BSS_STATUS].type = NLA_U32;

		nla_parse( tb, NL80211_ATTR_MAX, genlmsg_attrdata( gnlh, 0 ), genlmsg_attrlen( gnlh, 0 ), NULL );

		// Assume we're good to start with
		info->valid = true;

		if ( !tb[NL80211_ATTR_BSS] ||
			nla_parse_nested(bss, NL80211_BSS_MAX,
			tb[NL80211_ATTR_BSS], bss_policy ) )
		{
			info->valid = false;
			return NL_SKIP;
		}

		if( bss[NL80211_BSS_BSSID] )
			memcpy( info->bssid, nla_data( bss[NL80211_BSS_BSSID] ), 6 );
		else
			return NL_SKIP;

		if( bss[NL80211_BSS_STATUS] )
			info->bss_stat = (enum nl80211_bss_status)nla_get_u32( bss[NL80211_BSS_STATUS] );
		else
			return NL_SKIP;

		if( bss[NL80211_BSS_FREQUENCY] )
			info->freq = nla_get_u32( bss[NL80211_BSS_FREQUENCY] );

		// Get the essid
		const unsigned char *ie = (const unsigned char *)nla_data( bss[NL80211_BSS_INFORMATION_ELEMENTS] );
		int ielen = nla_len( bss[NL80211_BSS_INFORMATION_ELEMENTS] );
		char tmp[255] = { 0 };
		while( ielen >= 2 && ielen >= ie[1] )
		{
			if( ie[0] == 0 )
			{
				strncpy( tmp, (const char *)( ie + 2 ), ie[1] );
				info->essid = tmp;
				break;
			}
			ielen -= ie[1] + 2;
			ie += ie[1] + 2;
		}

		return NL_SKIP;
	}

	int WifiMon::get_iface_handler( struct nl_msg *msg, void *arg )
	{
		struct iface_info *info = (struct iface_info *)arg;
		struct genlmsghdr *gnlh = (struct genlmsghdr *)nlmsg_data( nlmsg_hdr( msg ) );
		struct nlattr *tb_msg[NL80211_ATTR_MAX + 1];

		nla_parse( tb_msg, NL80211_ATTR_MAX, genlmsg_attrdata( gnlh, 0 ), genlmsg_attrlen( gnlh, 0 ), NULL );

		// Assume we're good to start with
		info->valid = true;

		// Populate the stuff
		if ( tb_msg[NL80211_ATTR_IFNAME] )
			info->ifname = nla_get_string( tb_msg[NL80211_ATTR_IFNAME] );
		else
			info->valid = false;
		if( tb_msg[NL80211_ATTR_MAC] )
			memcpy( info->mac_addr, nla_data( tb_msg[NL80211_ATTR_MAC] ), 6 );
		else
			info->valid = false;
		if( tb_msg[NL80211_ATTR_IFTYPE] )
			info->iftype = (enum nl80211_iftype)nla_get_u32( tb_msg[NL80211_ATTR_IFTYPE] );
		else
			info->valid = false;
		if( tb_msg[NL80211_ATTR_WIPHY] )
			info->wiphy = nla_get_u32( tb_msg[NL80211_ATTR_WIPHY] );
		else
			info->valid = false;

		return NL_SKIP;
	}

	int WifiMon::error_handler( struct sockaddr_nl *nla, struct nlmsgerr *err, void *arg )
	{
		int *ret = (int *)arg;
		*ret = err->error;
		return NL_STOP;
	}

	int WifiMon::finish_handler( struct nl_msg *msg, void *arg )
	{
		int *ret = (int *)arg;
		*ret = 0;
		return NL_SKIP;
	}

	int WifiMon::ack_handler( struct nl_msg *msg, void *arg )
	{
		int *ret = (int *)arg;
		*ret = 0;
		return NL_STOP;
	}

	WifiMon::WifiMon( ) :
		nlh( NULL ),
		nl80211_id( -1 ),
		poll_interval( 5.0 )
	{
		nlh = nl_socket_alloc( );
		if( !nlh )
		{
			NODELET_FATAL( "Failed to allocate netlink socket." );
			ros::shutdown( );
		}

		if( genl_connect( nlh ) )
		{
			NODELET_FATAL( "Failed to connect to generic netlink." );
			ros::shutdown( );
		}

		nl80211_id = genl_ctrl_resolve( nlh, "nl80211" );
		if( nl80211_id < 0 )
		{
			NODELET_FATAL( "nl80211 not found." );
			ros::shutdown( );
		}
	}

	WifiMon::~WifiMon( )
	{
		nl_socket_free( nlh );
	}

	void WifiMon::onInit( )
	{
		nh = getNodeHandle( );
		nh_priv = getPrivateNodeHandle( );

		XmlRpc::XmlRpcValue xml_dev_list;
		xml_dev_list.setSize( 0 ); // This makes it TypeArray
		nh_priv.param( "poll_interval", poll_interval, 5.0 );
		nh_priv.param( "dev_list", xml_dev_list, xml_dev_list );
		ROS_ASSERT( xml_dev_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
		signed long long int dev_idx;
		if( xml_dev_list.size( ) )
		{
			for( unsigned int i = 0; i < xml_dev_list.size( ); i++ )
			{
				ROS_ASSERT( xml_dev_list[i].getType( ) == XmlRpc::XmlRpcValue::TypeString );
				std::string dev_name = xml_dev_list[i];
				dev_idx = if_nametoindex( dev_name.c_str( ) );
				if( dev_idx <= 0 )
					NODELET_WARN( "Device %s not found. Ignoring...", dev_name.c_str( ) );
				else
				{
					dev_ids.push_back( dev_idx );
					NODELET_DEBUG( "Found device %s at id %d.", dev_name.c_str( ), dev_idx );
				}
			}
		}
		NODELET_DEBUG( "Device detection complete; found %d devices.", dev_ids.size( ) );

		wifi_info_pub = nh.advertise<smd_wifi_monitor::WifiStatus>( "wifi_status", 1, false );
		PollCB( ros::TimerEvent( ) );
		poll_timer = nh_priv.createTimer( ros::Duration( poll_interval ), &WifiMon::PollCB, this );
	}

	void WifiMon::PollCB( const ros::TimerEvent &e )
	{
		ROS_DEBUG( "Polling all devices." );
		smd_wifi_monitor::WifiStatusPtr msg = smd_wifi_monitor::WifiStatusPtr( new smd_wifi_monitor::WifiStatus );
		msg->header.stamp = ros::Time::now( );
		for( unsigned int i = 0; i < dev_ids.size( ); i++ )
		{
			ROS_DEBUG( "Polling %d.", dev_ids[i] );
			struct iface_info info;
			if( get_info( dev_ids[i], &info ) )
			{
				ROS_WARN( "Failed to get info for netdev %d.", dev_ids[i] );
				continue;
			}
			smd_wifi_monitor::WifiDeviceInfo dev;
			dev.name = info.ifname;
			dev.mac = mac2str( info.mac_addr );
			dev.mode = iftype_name( info.iftype );
			dev.station.essid = info.essid;
			dev.station.bssid = mac2str( info.bssid );
			dev.station.frequency = info.freq;
			dev.station.state = bss_stat_name( info.bss_stat );
			dev.station.signal = info.signal;
			dev.station.bitrate = info.bitrate;
			dev.station.rx_pkts = info.rx_pkts;
			dev.station.tx_pkts = info.tx_pkts;
			dev.station.rx_bytes = info.rx_bytes;
			dev.station.tx_bytes = info.tx_bytes;
			msg->devices.push_back( dev );
		}
		wifi_info_pub.publish( msg );
	}

	int WifiMon::get_info( signed long long int dev_id, struct iface_info *info )
	{
		struct nl_msg *msg = NULL;
		struct nl_cb *cb;
		struct nl_cb *s_cb;
		int ret = 0;

		msg = nlmsg_alloc( );
		if( !msg )
		{
			NODELET_FATAL( "Failed to allocate netlink message." );
			ros::shutdown( );
		}

		cb = nl_cb_alloc( NL_CB_DEFAULT );
		s_cb = nl_cb_alloc( NL_CB_DEFAULT );
		if ( !cb || !s_cb )
		{
			NODELET_FATAL( "Failed to allocate netlink callbacks" );
			ros::shutdown( );
		}

		memset( &info->mac_addr, 0, 8 );
		info->iftype = (enum nl80211_iftype)((int)(NL80211_IFTYPE_MAX) + 1);
		info->wiphy = -1;
		memset( &info->bssid, 0, 8 );
		info->bss_stat = (enum nl80211_bss_status)4;
		info->freq = 0;
		info->rx_pkts = 0;
		info->tx_pkts = 0;
		info->rx_bytes = 0;
		info->tx_bytes = 0;
		info->signal = 0;
		info->bitrate = 0.0;

		/* GET_IFACE */
		genlmsg_put( msg, 0, 0, nl80211_id, 0, 0, NL80211_CMD_GET_INTERFACE, 0 );

		NLA_PUT_U32( msg, NL80211_ATTR_IFINDEX, dev_id );

		// HANDLER
		nl_cb_set( cb, NL_CB_VALID, NL_CB_CUSTOM, get_iface_handler, (void *)info );

		nl_socket_set_cb( nlh, s_cb );

		ret = nl_send_auto_complete( nlh, msg );
		if( ret < 0 )
			goto out;

		ret = 1;

		nl_cb_err( cb, NL_CB_CUSTOM, error_handler, &ret );
		nl_cb_set( cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &ret );
		nl_cb_set( cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &ret );

		while ( ret > 0 )
			nl_recvmsgs( nlh, cb );

		if( !info->valid )
		{
			ret = 1;
			goto out;
		}
		/* DONE WITH GET_IFACE */
		nlmsg_free( msg );
		msg = nlmsg_alloc( );
		if( !msg )
		{
			NODELET_FATAL( "Failed to allocate netlink message." );
			ros::shutdown( );
		}
		/* GET_SCAN */
		genlmsg_put( msg, 0, 0, nl80211_id, 0, NLM_F_DUMP, NL80211_CMD_GET_SCAN, 0 );

		NLA_PUT_U32( msg, NL80211_ATTR_IFINDEX, dev_id );

		// HANDLER
		nl_cb_set( cb, NL_CB_VALID, NL_CB_CUSTOM, get_link_bss_handler, (void *)info );

		nl_socket_set_cb( nlh, s_cb );

		ret = nl_send_auto_complete( nlh, msg );
		if( ret < 0 )
			goto out;

		ret = 1;

		nl_cb_err( cb, NL_CB_CUSTOM, error_handler, &ret );
		nl_cb_set( cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &ret );
		nl_cb_set( cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &ret );

		while ( ret > 0 )
			nl_recvmsgs( nlh, cb );

		if( !info->valid )
		{
			ret = 1;
			goto out;
		}
		/* DONE WITH GET_SCAN */
		if( info->bss_stat != NL80211_BSS_STATUS_ASSOCIATED )
			goto out;
		nlmsg_free( msg );
		msg = nlmsg_alloc( );
		if( !msg )
		{
			NODELET_FATAL( "Failed to allocate netlink message." );
			ros::shutdown( );
		}
		/* GET_STATION */
		genlmsg_put( msg, 0, 0, nl80211_id, 0, 0, NL80211_CMD_GET_STATION, 0 );

		NLA_PUT_U32( msg, NL80211_ATTR_IFINDEX, dev_id );

		// HANDLER
		NLA_PUT( msg, NL80211_ATTR_MAC, 6, info->bssid );
		nl_cb_set( cb, NL_CB_VALID, NL_CB_CUSTOM, get_link_sta_handler, (void *)info );

		nl_socket_set_cb( nlh, s_cb );

		ret = nl_send_auto_complete( nlh, msg );
		if( ret < 0 )
			goto out;

		ret = 1;

		nl_cb_err( cb, NL_CB_CUSTOM, error_handler, &ret );
		nl_cb_set( cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &ret );
		nl_cb_set( cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &ret );

		while ( ret > 0 )
			nl_recvmsgs( nlh, cb );

		if( !info->valid )
		{
			ret = 1;
			goto out;
		}
		// Just in case we lost the bssid in the scan, we don't
		// want to discard all results...
		ret = 0;
		/* DONE WITH GET_STATION */

		out:
		nl_cb_put( cb );
		out_free_msg:
		nlmsg_free( msg );

		return ret;

		nla_put_failure:
		NODELET_ERROR( "Building message failed." );
	}
}
