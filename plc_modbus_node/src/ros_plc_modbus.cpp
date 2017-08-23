//
// Created by Brad Bazemore on 10/29/15.
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <modbus/modbus.h>

class plc_modbus_manager {
public:
    plc_modbus_manager();

private:
    ros::NodeHandle node;

    ros::Publisher regs_read;
    ros::Subscriber regs_write;
    ros::Publisher coils_read;
    ros::Subscriber coils_write;
    ros::Publisher regs_input_read;

    std::vector<int> regs_addr;
    std::vector<int> coils_addr;
    std::vector<int> regs_input_addr;

    std_msgs::UInt16MultiArray regs_val;
    std_msgs::ByteMultiArray coils_val;
    std_msgs::UInt16MultiArray regs_input_val;

    modbus_t *plc;

    std::string ip_address, device;
    int port, baudrate, bit_num, stop_bit;
    char parity;
    int spin_rate;
    int slave_id;

    void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data);

    void coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data);
};


plc_modbus_manager::plc_modbus_manager() {

    regs_read = node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_read", 100);
    regs_write = node.subscribe<std_msgs::UInt16MultiArray>("modbus/regs_write", 100,
                                                            &plc_modbus_manager::regs_callBack, this);
    coils_read = node.advertise<std_msgs::ByteMultiArray>("modbus/coils_read", 100);
    coils_write = node.subscribe<std_msgs::ByteMultiArray>("modbus/coils_write", 100,
                                                           &plc_modbus_manager::coils_callBack, this);
    regs_input_read = node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_input_read", 100);

    node.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.0.100");
    node.param("plc_modbus_node/port", port, 502);
    node.param("plc_modbus_node/spin_rate", spin_rate, 30);
    node.param<std::string>("plc_modbus_node/device", device, "");
    node.param("plc_modbus_node/baudrate", baudrate, 115200);
    node.param("plc_modbus_node/bit_num", bit_num, 8);
    //node.param("plc_modbus_node/parity", parity, 'N');
    node.param("plc_modbus_node/stop_bit", stop_bit, 1);
    node.param("plc_modbus_node/slave_id", slave_id, 0x01);
    parity = 'N';

    if (!node.getParam("plc_modbus_node/regs_addr", regs_addr)) {
        ROS_WARN("No reg addrs given!");
    }
    if (!node.getParam("plc_modbus_node/coils_addr", coils_addr)) {
        ROS_WARN("No coil addrs given!");
    }
    if (!node.getParam("plc_modbus_node/regs_input_addr", regs_input_addr)) {
        ROS_WARN("No reg input addrs given!");
    }

    if (device.empty()){
      ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
      plc = modbus_new_tcp(ip_address.c_str(), port);
    }else{
      ROS_INFO("Connecting to modbus device on %s %d:%d:%c:%d", device.c_str(), baudrate, bit_num, parity, stop_bit);
      plc = modbus_new_rtu(device.c_str(), baudrate, parity, bit_num, stop_bit);
    }
    if (plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }

    struct timeval t;
    t.tv_sec = 1;
    t.tv_usec = 200000;
    modbus_set_response_timeout(plc, &t);
    // set debug mode
    modbus_set_debug(plc, TRUE);

    //modbus_rtu_set_serial_mode(plc, MODBUS_RTU_RS485);
    //modbus_set_error_recovery(plc, MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL );

    if (!device.empty() && modbus_set_slave(plc, slave_id) == -1) {
        ROS_FATAL("Failed to connect to %04x slave!!!", slave_id);
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(plc);
        return;
    } else {
        ROS_INFO("Connection to %04x slave", slave_id);
    }

    if (modbus_connect(plc) == -1) {
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(plc);
        return;
    } else {
        ROS_INFO("Connection to modbus device established");
    }

    ros::Rate loop_rate(spin_rate);

    while (ros::ok()) {
        regs_val.data.clear();
        coils_val.data.clear();
        regs_input_val.data.clear();

        for (int i = 0; i < regs_addr.size(); i++) {
            uint16_t temp[1] = {0};
            if (modbus_read_registers(plc, regs_addr.at(i), 1, temp) == -1) {
                ROS_ERROR("Unable to read reg addr: %04x", regs_addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                regs_val.data.push_back(temp[0]);
            }
        }
        if (regs_val.data.size() > 0) {
            regs_read.publish(regs_val);
        }

        for (int i = 0; i < coils_addr.size(); i++) {
            uint8_t temp[1] = {0};
            if (modbus_read_bits(plc, coils_addr.at(i), 1, temp) == -1) {
                ROS_ERROR("Unable to read coil addr: %04x", coils_addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                coils_val.data.push_back(temp[0]);
            }
        }
        if (coils_val.data.size() > 0) {
            coils_read.publish(coils_val);
        }

        for (int i = 0; i < regs_input_addr.size(); i++) {
            uint16_t temp[1] = {0};
            if (modbus_read_input_registers(plc, regs_input_addr.at(i), 1, temp) == -1) {
                ROS_ERROR("Unable to read input_reg addr: %04x", regs_input_addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                regs_input_val.data.push_back(temp[0]);
            }
        }
        if (regs_input_val.data.size() > 0) {
            regs_input_read.publish(regs_val);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    modbus_close(plc);
    modbus_free(plc);
    return;
}

void plc_modbus_manager::regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data) {
    if (regs_data->data.size() != regs_addr.size()) {
        ROS_ERROR("%d registers to write but only %d given!", regs_addr.size(), regs_data->data.size());
        return;
    }
    for (int i = 0; i < regs_data->data.size(); i++) {
        ROS_DEBUG("regs_out[%d]:%u", i, regs_data->data.at(i));
        uint16_t temp[1] = {regs_data->data.at(i)};
        if (modbus_write_registers(plc, regs_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", regs_addr.at(i), regs_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus register write at addr:%d with value:%u", regs_addr.at(i), regs_data->data.at(i));
        }
    }
}

void plc_modbus_manager::coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data) {
    if (coils_data->data.size() != coils_addr.size()) {
        ROS_ERROR("%d coils to write but %d given!", coils_addr.size(), coils_data->data.size());
        return;
    }
    for (int i = 0; i < coils_data->data.size(); i++) {
        ROS_DEBUG("regs_out[%d]:%u", i, coils_data->data.at(i));
        uint8_t temp[1] = {coils_data->data.at(i)};
        if (modbus_write_bits(plc, coils_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus coil write failed at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus coil write at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_plc_modbus");
    plc_modbus_manager mm;
    return 0;
}
