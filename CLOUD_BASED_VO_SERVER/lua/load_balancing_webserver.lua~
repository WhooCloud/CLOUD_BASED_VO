local server = require "resty.websocket.server"
local cjson = require "cjson.safe"
local ffi = require "ffi"

if pcall(ffi.typeof, "struct timeval") then
    -- do nothing
else
ffi.cdef[[
    struct timeval{
        long int tv_sec;
        long int tv_usec;
    };

    int gettimeofday(struct timeval *tv, void *tz);
]]
end

if pcall(function () return ffi.C.gettimeofday end) then
    -- do nothing
else
ffi.cdef[[
    int gettimeofday(struct timeval *tv, void *tz);
]]
end


local function get_current_time()
    local tm = ffi.new("struct timeval")   
    ffi.C.gettimeofday(tm, nil)
    --print(string.format("%.6f",sec + usec * 10^-6))
    return tonumber(tm.tv_sec) + tonumber(tm.tv_usec) * 10^-6
end


local function receive_text(wb)
    local data, typ, err = wb:recv_frame()
    if not data then
        ngx.log(ngx.ERR, "failed to receive a frame: ", err)
        return ngx.exit(444)
    end
    if typ == "text" then
        --ngx.log(ngx.INFO, "received a text frame: ", data);
        return data
    else
        ngx.log(ngx.ERR, "failed to receive a text, ", "the type is: ", typ)
        return nil
    end                                                                         
end

local function delay_ms(ms)
    local time1 = get_current_time()
    while ((get_current_time() - time1) < 0.001 * ms)
    do
        --pass
    end
    return
end



local wb, err = server:new{
    --timeout = 30,  -- in milliseconds
    max_payload_len = 200*1024,
}
if not wb then
    ngx.log(ngx.ERR, "failed to new websocket: ", err)
    return ngx.exit(444)
end

while(true)
do
    local data = receive_text(wb)

    local time_current = get_current_time()
    local json_state_imu = cjson.decode(data)
    local time_previous = json_state_imu.timestamp
    local no=json_state_imu.NO

    ngx.log(ngx.INFO, "NO: ", no, " delay1 is: ", string.format("%.3f",(time_current - time_previous)*1000), " ms")
 --   wb:set_timeout(1000)  -- change the network timeout to 1 second
    delay_ms(1000)
    local json_state_fused = json_state_imu
    json_state_fused.timestamp = get_current_time()
    local data_send = cjson.encode(json_state_fused)

    local bytes, err = wb:send_text(data_send)
    if not bytes then
        ngx.log(ngx.ERR, "failed to send a text frame: ", err)
        return ngx.exit(444)
    end
end


--[[local bytes, err = wb:send_close(1000, "enough, enough!")
if not bytes then
    ngx.log(ngx.ERR, "failed to send the close frame: ", err)
    return
end]]--

