#ifndef motoman_mh12_TYPES_HPP

#include<vector>
#include<base/Time.hpp>

namespace motoman_mh12 {
    struct GPIO
    {   
        int address;
        int value;
    };
    struct GPIOs
    {
        std::vector<GPIO> ports;
        base::Time timestamp;
    };
}
#endif

