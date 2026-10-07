#include <cassert>
#include "../../firmware/receive_freshness.h"
using namespace antenna_controller;

int main() {
    ReceiveFreshness node1, node2, sensor;
    constexpr uint32_t timeout = 5000;
    // No phantom ONLINE at boot, including timestamp zero.
    assert(!node1.fresh(0, timeout));
    assert(!node1.fresh(1000, timeout));
    assert(!node1.update(5000, timeout));
    node1.receive(0);
    assert(node1.fresh(0, timeout));
    assert(node1.update(0, timeout));  // redraw for first receipt
    assert(!node1.update(4999, timeout));
    assert(node1.fresh(4999, timeout));
    assert(node1.update(5000, timeout));  // redraw without another packet
    assert(!node1.fresh(5000, timeout));
    assert(!node1.update(5001, timeout));
    assert(!node1.update(0, timeout));  // a later millis wrap cannot revive it

    // One silent node must expire even while the other continues reporting.
    node1.receive(10000);
    node2.receive(10000);
    assert(node1.update(10000, timeout));
    assert(node2.update(10000, timeout));
    node2.receive(14999);
    assert(node1.update(15000, timeout));
    assert(!node2.update(15000, timeout));
    assert(!node1.fresh(15000, timeout));
    assert(node2.fresh(15000, timeout));
    node1.receive(15001);
    assert(node1.update(15001, timeout));  // redraw on recovery

    // Raw sensor values expire before the overall link, even without new data.
    sensor.receive(15001);
    assert(sensor.update(15001, 3000));
    assert(sensor.update(18001, 3000));
    assert(node1.fresh(18001, timeout));

    ReceiveFreshness wrapping;
    wrapping.receive(UINT32_MAX - 1000);
    assert(wrapping.update(UINT32_MAX - 1000, timeout));
    assert(wrapping.fresh(3998, timeout));
    assert(wrapping.update(3999, timeout));
    assert(!wrapping.fresh(3999, timeout));
}
