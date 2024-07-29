#pragma once

#include <string>
#include <sstream>
#include <cstring>
#include <iostream>

/* Storage for testing, TODO: "Data must be stored in the type defined in UDP packet." */

namespace udp
{

    union Value
    {
        bool value_bool;
        int8_t value_int8;
        int16_t value_int16;
        int32_t value_int32;
        int64_t value_int64;
        uint8_t value_uint8;
        uint16_t value_uint16;
        uint32_t value_uint32;
        uint64_t value_uint64;
        float value_float;
        double value_double;
        char value_char;
    };

    struct __attribute__((__packed__)) Packet
    {
        uint8_t sensorId;
        uint8_t dataType;
        Value value;

        std::string toString() const
        {
            std::ostringstream oss;
            oss << "Sensor ID: " << static_cast<int>(sensorId)
                << ", Data Type: " << static_cast<int>(dataType) << ", Value: ";
            switch (dataType)
            {
            case 0:
                oss << value.value_bool;
                break;
            case 1:
                oss << static_cast<int>(value.value_int8);
                break;
            case 2:
                oss << value.value_int16;
                break;
            case 3:
                oss << value.value_int32;
                break;
            case 4:
                oss << value.value_int64;
                break;
            case 5:
                oss << static_cast<unsigned int>(value.value_uint8);
                break;
            case 6:
                oss << value.value_uint16;
                break;
            case 7:
                oss << value.value_uint32;
                break;
            case 8:
                oss << value.value_uint64;
                break;
            case 9:
                oss << value.value_float;
                break;
            case 10:
                oss << value.value_double;
                break;
            case 11:
                oss << value.value_char;
                break;
            default:
                oss << "Unknown";
                break;
            }
            return oss.str();
        }

        Packet &operator=(const char *buffer)
        {
            size_t offset = 0;

            sensorId = buffer[offset];
            offset += sizeof(sensorId);

            dataType = buffer[offset];
            offset += sizeof(dataType);

            switch (dataType)
            {
            case 0:
                value.value_bool = buffer[offset];
                break;
            case 1:
                value.value_int8 = buffer[offset];
                break;
            case 2:
                std::memcpy(&value.value_int16, buffer + offset,
                            sizeof(value.value_int16));
                break;
            case 3:
                std::memcpy(&value.value_int32, buffer + offset,
                            sizeof(value.value_int32));
                break;
            case 4:
                std::memcpy(&value.value_int64, buffer + offset,
                            sizeof(value.value_int64));
                break;
            case 5:
                value.value_uint8 = buffer[offset];
                break;
            case 6:
                std::memcpy(&value.value_uint16, buffer + offset,
                            sizeof(value.value_uint16));
                break;
            case 7:
                std::memcpy(&value.value_uint32, buffer + offset,
                            sizeof(value.value_uint32));
                break;
            case 8:
                std::memcpy(&value.value_uint64, buffer + offset,
                            sizeof(value.value_uint64));
                break;
            case 9:
                std::memcpy(&value.value_float, buffer + offset,
                            sizeof(value.value_float));
                break;
            case 10:
                std::memcpy(&value.value_double, buffer + offset,
                            sizeof(value.value_double));
                break;
            case 11:
                value.value_char = buffer[offset];
                break;
            default:
                std::cerr << "Unknown data type" << std::endl;
                break;
            }

            return *this;
        }

    double getValueAsDouble()
    {
        switch (dataType)
        {
        case 0:
            return static_cast<double>(value.value_bool);
        case 1:
            return static_cast<double>(value.value_int8);
        case 2:
            return static_cast<double>(value.value_int16);
        case 3:
            return static_cast<double>(value.value_int32);
        case 4:
            return static_cast<double>(value.value_int64);
        case 5:
            return static_cast<double>(value.value_uint8);
        case 6:
            return static_cast<double>(value.value_uint16);
        case 7:
            return static_cast<double>(value.value_uint32);
        case 8:
            return static_cast<double>(value.value_uint64);
        case 9:
            return static_cast<double>(value.value_float);
        case 10:
            return value.value_double;
        case 11:
            return static_cast<double>(value.value_char);
        default:
            std::cerr << "Unknown data type" << std::endl;
            return 0.0;
        }
    }
    };

} // namespace udp
