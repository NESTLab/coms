from __future__ import annotations
from coms.utils import Device
from coms.listener import Listener
from coms.broadcaster import Broadcaster
from coms.map_subscriber import MapSubscriber


class Wifi(Device):
    def __init__(self: Wifi, network_interface: str, namespace: str, map_sub: MapSubscriber) -> None:
        super().__init__(network_interface)
        self.namespace = namespace
        self.listener = Listener(
            address=(self.local_ip_addr, Device._get_port(self.local_ip_addr, outgoing=False)),
            namespace=namespace)
        self.broadcaster = Broadcaster(
            address=(self.local_ip_addr, Device._get_port(self.local_ip_addr, outgoing=True)),
            namespace=namespace,
            map_sub=map_sub)

    # Called to begin communications
    def start(self: Wifi) -> None:
        print("Setting up WiFi device")
        self.listener.start()
        self.broadcaster.start()

    # Called to terminate communications
    def stop(self: Wifi) -> None:
        print("Shutting down WiFi device")
        self.listener.stop()
        self.broadcaster.stop()


if __name__ == "__main__":
    wifi = Wifi('wlan0', 'red')
    wifi.start()
    input("press enter to stop")
