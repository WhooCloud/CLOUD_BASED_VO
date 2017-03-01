local ffi = require("ffi")
local slamBase = ffi.load("libslamBase.so")

if pcall(function () return slamBase.showImage end) then
    -- do nothing
else
ffi.cdef[[
    char* showImage(int index);
]]
end

local rows = slamBase.showImage(1)
print(rows)
print(ffi.string(rows))

