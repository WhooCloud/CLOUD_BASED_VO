local ffi = require("ffi")
local slamInterface = ffi.load("libslamInterface.so")

if pcall(function () return slamInterface.FFIInterface end) then
    -- do nothing
else
ffi.cdef[[
	char* FFIInterface(const char* data);
]]
end

local data = "{\"type\" : \"mainloop\", \"pts_img\" : \"155,253.6;253,156;153.3,175.2;\", \"pts_obj\" : \"155,253.6,11;253,156,22.2;153.3,175.2,33.3;\", \"camera_cx\" : 325.5, \"camera_cy\" : 253.5, \"camera_fx\" : 518.0, \"camera_fy\" : 519.0, \"camera_scale\" : 1000.0 }"
local result = slamInterface.FFIInterface(data)
print(ffi.string(result))

