local ffi = require "ffi"
ffi.cdef[[
    struct timeval{
        long int tv_sec;
        long int tv_usec;
    };

    int gettimeofday(struct timeval *tv, void *tz);
]];

local tm = ffi.new("struct timeval")    
function get_current_time()

    ffi.C.gettimeofday(tm, nil)
    local sec = ()
    local usec = (tm.tv_usec)
    --print(string.format("%.6f",sec + usec * 10^-6))
    return sec + usec * 10^-6
end

--get_current_time()
