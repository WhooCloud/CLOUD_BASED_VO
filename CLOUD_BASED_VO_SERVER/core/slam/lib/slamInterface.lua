--local ffi = require("ffi")
--local slamInterface = ffi.load("libslamInterface.so")

--if pcall(function () return slamInterface.FFIInterface end) then
--    -- do nothing
--else
--ffi.cdef[[
--	char* FFIInterface(const char* data);
--	char* FFICompressString(const char* data);
--	char* FFIDecompressString(const char* data);
--]]
--end

--local data = "HelloWorldHelloWorldHelloWorldHelloWorld"
--local Cdata = slamInterface.FFICompressString(data);
--print(ffi.string(Cdata))
--local Ddata = slamInterface.FFIDecompressString(Cdata);
--print(ffi.string(Ddata))
--local result = slamInterface.FFIInterface(data)

local ffi = require("ffi")
ffi.cdef[[
unsigned long compressBound(unsigned long sourceLen);
int compress2(uint8_t *dest, unsigned long *destLen,
	      const uint8_t *source, unsigned long sourceLen, int level);
int uncompress(uint8_t *dest, unsigned long *destLen,
	       const uint8_t *source, unsigned long sourceLen);
]]
local zlib = ffi.load(ffi.os == "Windows" and "zlib1" or "z")

local function compress(txt)
  local n = zlib.compressBound(#txt)
  local buf = ffi.new("uint8_t[?]", n)
  local buflen = ffi.new("unsigned long[1]", n)
  local res = zlib.compress2(buf, buflen, txt, #txt, 9)
  assert(res == 0)
  return ffi.string(buf, buflen[0])
end

local function uncompress(comp, n)
  local buf = ffi.new("uint8_t[?]", n)
  local buflen = ffi.new("unsigned long[1]", n)
  local res = zlib.uncompress(buf, buflen, comp, #comp)
  assert(res == 0)
  return ffi.string(buf, buflen[0])
end

-- Simple test code.
local txt = string.rep("abcd", 1000)
print("Uncompressed size: ", #txt)
local c = compress(txt)
print("Compressed size: ", #c)
local txt2 = uncompress(c, #txt)
assert(txt2 == txt)