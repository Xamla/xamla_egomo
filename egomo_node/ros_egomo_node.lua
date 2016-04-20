local ros = require("ros")
local path = require("pl.path")
local posix = require("posix")

-- Gripper commands--

-- reset the gripper with falling edge 1.0 --> 0.0
local GRIPPER_RESET = "gripper0.reset"
-- sets the position of the gripper in mm (range?)
local GRIPPER_POS_CMD = "gripper0.pos_cmd"
-- sets the gripper's maximum force in N (range?)
local GRIPPER_MAX_FORCE = "gripper0.max_force"
-- sets the gripepr's maximum speed in mm/s (range?)
local GRIPPER_MAX_SPEED = "gripper0.max_speed"
-- returns the gripper's state
local GRIPPER_STATE = "gripper0.state"
-- returns the gripper's current position information
local GRIPPER_POS_FB = "gripper0.pos_fb"
-- sets the gripper's force in N
local GRIPPER_GRIP_FORCE = "gripper0.grip_force"
-- sets the gripper's left finger force in N
local GRIPPER_LEFT_FINGER_FORCE = "gripper0.left_finger_force"
-- sets the gripper's right finger force in N
local GRIPPER_RIGHT_FINGER_FORCE = "gripper0.right_finger_force"

-- Force Torque commands--

-- reset the force torque (ft) sensor with falling edge 1.0 --> 0.0
local FT_RESET = "ft0.reset"
-- returns the ft's state
local FT_STATE = "ft0.state"
-- returns the force in x direction (in N)
local FT_X = "ft0.x"
-- returns the force in y direction (in N)
local FT_Y = "ft0.y"
-- returns the force in z direction (in N)
local FT_Z = "ft0.z"
-- returns the torque of the x (roll) axis (in Nm)
local FT_A = "ft0.a"
-- returns the torque of the y (pitch) axis (in Nm)
local FT_B = "ft0.b"
-- returns the torque of the z (yaw) (in Nm)
local FT_C = "ft0.c"

-- IMU commands --

-- reset the imu with falling edge 1.0 --> 0.0
local IMU_RESET = "imu0.reset"
-- returns the imu's state
local IMU_STATE = "imu0.state"
-- returns the acceleration in the x-axis (roll) (in m/s^2)
local IMU_A = "imu0.a"
-- returns the acceleration in the y-axis (pitch) (in m/s^2)
local IMU_B = "imu0.b"
-- returns the acceleration in the z-axis (yaw) (in m/s^2)
local IMU_C = "imu0.c"
-- returns the angular velocity around the x-axis (in rad/s)
local IMU_DA = "imu0.da"
-- returns the angular velocity around the y-axis (pitch) (in rad/s)
local IMU_DB = "imu0.db"
-- returns the angular velocity around the z-axis (yaw) (in rad/s)
local IMU_DC = "imu0.dc"

-- ros common
local spinner
local nodehandle

-- forward declaration
local enqueue

-- Gripper --
local gripper_spec
local publisher_gripper
local service
local commands_for_gripper = {}
local service_queue
local send_set_command_spec

-- Force Torque --
local publisher_force_torque
local force_torque_spec
local commands_for_force_torque = {}

-- IMU --
local imu_spec
local publisher_imu
local commands_for_imu = {}

-- Identificaton constants
local GRIPPER = 1
local FORCE_TORQUE = 2
local IMU = 3

local file_descriptor = nil

local debug = true

-- Buffer for data which are supposed to be written to file/device
local write_buffer = {}
-- Buffer for data which have been read from the file/device
local read_buffer = {}

local device = "/dev/ttyACM0"

-- Add all commands for the gripper to a table: Is used in order to fill the message
local function add_commands_for_gripper()
  commands_for_gripper = {
    GRIPPER_RESET,
    GRIPPER_POS_CMD,
    GRIPPER_MAX_FORCE,
    GRIPPER_MAX_SPEED,
    GRIPPER_STATE,
    GRIPPER_POS_FB,
    GRIPPER_GRIP_FORCE,
    GRIPPER_LEFT_FINGER_FORCE,
    GRIPPER_RIGHT_FINGER_FORCE
  }
end

-- Add all commands for the force torque to a table: Is used in order to fill the message
local function add_commands_for_force_torque()
  commands_for_force_torque = {
    FT_RESET,
    FT_STATE,
    FT_X,
    FT_Y,
    FT_Z,
    FT_A,
    FT_B,
    FT_C
  }
end

-- Add all commands for the IMU to a table: Is used in order to fill the message
local function add_commands_for_imu()
  commands_for_imu = {
    IMU_RESET,
    IMU_STATE,
    IMU_A,
    IMU_B,
    IMU_C,
    IMU_DA,
    IMU_DB,
    IMU_DC
  }
end


-- This function searches for devices in order to get possible paths for read/write

-- Input:
--  id_vendor - The vendor id of the desired device
--  id_prdocut - The product id of the desired device
-- Output:
--  found - Table with all connected devices (e.g. {"/dev/ttyACM0", "/dev/ttyACM1"}
local function find_tty_acm(id_vendor, id_product)
  local function read_first_line(fn)
    local file = io.open(fn, 'rb')
    local ln
    if file ~= nil then
      ln = file:read('*l')
      file:close()
    end
    return ln
  end

  function starts_with(string, start)
    return string.sub(string, 1, #start) == start
  end

  local found = {}
  local USB_DEVICES = '/sys/bus/usb/devices'
  for dnbase in path.dir(USB_DEVICES) do
    local dn = path.join(USB_DEVICES, dnbase)
    local vid_fn = path.join(dn, 'idVendor')
    local pid_fn = path.join(dn, 'idProduct')
    if path.exists(vid_fn) and path.exists(pid_fn) then
      if id_vendor == read_first_line(vid_fn) and id_product == read_first_line(pid_fn) then
        for subdir in path.dir(dn) do
          if starts_with(subdir, dnbase .. ':') then
            for subsubdir in path.dir(path.join(dn, subdir)) do
              if starts_with(subsubdir, 'tty') then
                for subsubsubdir in path.dir(path.join(dn, subdir .. "/" ..subsubdir)) do
                  if starts_with(subsubsubdir,"ttyACM") then
                    table.insert(found, path.join('/dev', subsubsubdir))
                  end
                end
              end
            end
          end
        end
      end
    end
  end
  return found
end

-- This function searches for egomo devices

-- Output:
--  devices - Table containing all devices
local function get_egomo_devices()
  local vendor_id = "0483"
  local product_id = "5740"

  return find_tty_acm(vendor_id, product_id)
end

-- This function finds the corresponding command for the service

-- Input:
--  command - Command which was send from service caller (e.g. "reset")
--  command_list - List containg commands for search
-- Ouput:
--  v - The corresponding command (e.g. "gripper0.reset")
local function choose_command(command, command_list)
  for i,v in ipairs(command_list) do
    if string.find(v,command) then
      return v
    end
  end
  return nil
end

-- Handler for service SendSetCommand
local function send_set_command_handler(request, response, header)
  local command_to_send = choose_command(request.command_name, commands_for_gripper)

  if command_to_send ~= nil then
    enqueue(command_to_send, request.value)
    response.response = "command accepted"
  else
    response.response = "command not valid"
    return false
  end
  return true
end

-- Function in order to do initialization used components (e.g. Services)
local function init()
  ros.init('xamla_egomo')
  spinner = ros.AsyncSpinner()
  spinner:start()
  nodehandle = ros.NodeHandle()

  -- Gripper --
  add_commands_for_gripper()
  gripper_spec = ros.MsgSpec('xamla_egomo/XamlaGripper')
  publisher_gripper= nodehandle:advertise("XamlaGripper", gripper_spec)

  send_set_command_spec = ros.SrvSpec('xamla_egomo/SendGripperSetCommand')

  service_queue = ros.CallbackQueue()
  service = nodehandle:advertiseService('xamla_egomo/SendGripperSetCommand', send_set_command_spec, service_queue, send_set_command_handler)

  -- Force Torque --
  add_commands_for_force_torque()
  force_torque_spec = ros.MsgSpec('geometry_msgs/WrenchStamped')
  publisher_force_torque = nodehandle:advertise("XamlaForceTorque", force_torque_spec)

  -- IMU --
  add_commands_for_imu()
  imu_spec = ros.MsgSpec('geometry_msgs/AccelStamped')
  publisher_imu = nodehandle:advertise("XamlaIOIMU", imu_spec)
end

-- This function assigns the a value to a field of a message (if possible)

-- Input:
--  message - The message the value is supposed to be assigned to
--  fieldName - The name of the field to which the value is supposed to be assigned to
--  value - The value which is supposed to be assigned
--  type - The type of the message (GRIPPER = 1, FORCETORQUE = 2, IMU = 3)
-- Output:
--  true if value was assigned; false otherwise
local function assign_values_to_message(message, fieldName, value, type)
  if type == GRIPPER then
    if fieldName == GRIPPER_GRIP_FORCE then
      message.grip_force = value
      return true
    elseif fieldName == GRIPPER_POS_FB then
      message.pos_fb = value
      return true
    elseif fieldName == GRIPPER_LEFT_FINGER_FORCE then
      message.left_finger_force = value
      return true
    elseif fieldName == GRIPPER_RIGHT_FINGER_FORCE then
      message.right_finger_force = value
      return true
    else
      return false
    end
  elseif type == FORCE_TORQUE then
    if fieldName == FT_X then
      message.wrench.force.x = value
      return true
    elseif fieldName == FT_Y then
      message.wrench.force.y = value
      return true
    elseif fieldName == FT_Z then
      message.wrench.force.z = value
      return true
    elseif fieldName == FT_A then
      message.wrench.torque.x = value
      return true
    elseif fieldName == FT_B then
      message.wrench.torque.y = value
      return true
    elseif fieldName == FT_C then
      message.wrench.torque.z = value
      return true
    else
      return false
    end
  elseif type == IMU then
    if fieldName == IMU_A then
      message.accel.linear.x = value
      return true
    elseif fieldName == IMU_B then
      message.accel.linear.y = value
      return true
    elseif fieldName == IMU_C then
      message.accel.linear.z = value
      return true
    elseif fieldName == IMU_DA then
      message.accel.angular.x = value
      return true
    elseif fieldName == IMU_DB then
      message.accel.angular.y = value
      return true
    elseif fieldName == IMU_DC then
      message.accel.angular.z = value
      return true
    else
      return false
    end
  end
  return false
end

local function has_value(table, value)
  for i, v in ipairs(table) do
    if v == value then
      return true
    end
  end
  return false
end

-- This function is used to parse a read line. Returns nil if the
-- line does not contain a defined command

-- Input:
--  line - String which will be used to find a specific known command (e.g. "gripper0.pos_fb <= gripper0.pos_fb = 1.004000")
--  commands - List of commands which are used for comparision
--  blacklist - Commands that should be skipped (e.g. gripper0.pos_fb)
-- Output:
--  fieldName - Name of the field corresponds to this value (e.g. "gripper0.pos_fb")
--  value - The corresponding value as a number (e.g. 1.004)
local function parse_data(line, commands, blacklist) -- TODO: Konzept mit blacklist diskutieren und ggf. optional machen
  local fieldName
  local value

  for i,v in ipairs(commands) do
    local first, last = string.find(line, v .. " = ")
    if first ~= nil then
      if has_value(blacklist, v) then
        return nil
      else
        fieldName = v
        value = string.sub(line,last,string.len(line))
        return fieldName, tonumber(value)
      end
    end
  end
  return nil
end

local function type_to_string(type)
  if type == GRIPPER then
    return "GRIPPER"
  elseif type == FORCE_TORQUE then
    return "FORCE_TORQUE"
  elseif type == IMU then
    return "IMU"
  else
    return "ERROR: UNKNOWN TYPE!"
  end
end

-- This function is responsible for creating and publishing the messages based on the read data

-- Input:
--  type - Defines the type of the message (e.g. GRIPPER, FORCE_TORQUE, IMU)
local function build_xamla_message(type)
  local message
  local MESSAGE_PARAMETERS
  local used_command_list
  local used_publisher
  local numberOfSetParameters = 0
  local seq = 0
  local blacklist = {}

  -- Initialize components based on type
  if type == GRIPPER then
    message = ros.Message('xamla_egomo/XamlaGripper')
    MESSAGE_PARAMETERS = 4 -- table.getn(message.spec.fields) TODO: geht nicht mehr bei Verschachtelung
    used_command_list = commands_for_gripper
    used_publisher = publisher_gripper
  elseif type == FORCE_TORQUE then
    message = ros.Message('geometry_msgs/WrenchStamped')
    MESSAGE_PARAMETERS = 6 -- table.getn(message.spec.fields)
    used_command_list = commands_for_force_torque
    used_publisher = publisher_force_torque
  elseif type == IMU then
    message = ros.Message('geometry_msgs/AccelStamped')
    MESSAGE_PARAMETERS = 6 -- table.getn(message.spec.fields)
    used_command_list = commands_for_imu
    used_publisher = publisher_imu
  end
  
  message.header.frame_id = "ee_link"

  while true do
    for i,line in ipairs(read_buffer) do
      -- Skip line if it is answer of an previous set command
      if string.match(string.sub(line,1,2),"OK") == nil then
        -- Inspect the line and look for corresponding field and value
        local field_name, value = parse_data(line, used_command_list, blacklist)
        if field_name ~= nil and value ~= nil then
          -- Try to assign the value to the given field
          if assign_values_to_message(message, field_name, value, type) then
            table.insert(blacklist, field_name)
            numberOfSetParameters = numberOfSetParameters + 1;
          end
        end
      end
    end

    -- If message is complete publish message
    if numberOfSetParameters == MESSAGE_PARAMETERS then
      message.header.seq = seq
      message.header.stamp = ros.Time.now()
      used_publisher:publish(message)
      seq = seq + 1
      blacklist = {}
      numberOfSetParameters = 0
      ros.DEBUG("XAMLA_" .. type_to_string(type) .. ": Message send yield now")
      coroutine.yield()
    else
      ros.DEBUG("XAMLA_" .. type_to_string(type) .. ": Message unfinished yield now")
      coroutine.yield()
    end
  end
end

-- This function adds some signs to the command in order to meet the defined communication protocol
local function build_complete_command(command, value)
  local complete_msg
  if value ~= nil then
    complete_msg = command .. " = " .. string.format("%.1f",value) .."\n"
  else
    complete_msg = command .."\n"
  end
  return complete_msg
end

-- This function enqueue a command into the write buffer
-- Input:
--  command - The command that should be send (e.g. gripper0.pos_cmd)
--  value - The value for the command (e.g. 1.0 or nil)
function enqueue(command, value)
  local complete_msg = build_complete_command(command, value)
  table.insert(write_buffer, complete_msg)
end

-- This function opens the file/device for reading and writing

-- Input:
--  device - path or device (e.g. "/dev/ttyACM0")
local function init_file_descriptor(device, error_on_read_write)

  -- Scans for devices of type egomo IO board
  local devices = get_egomo_devices()

  if #devices ~= 0 then
    if file_descriptor == nil or file_descriptor == -1 or error_on_read_write then
      if file_descriptor ~= nil and file_descriptor ~= -1 then
        posix.close(file_descriptor)
      end
      -- Run through all devices and try to open it
      for i, v in ipairs(devices) do
        file_descriptor, err = posix.open(v, posix.O_RDWR + posix.O_NONBLOCK);
        -- Set device if open was successful and break loop
        if err ~= nil then
          ros.ERROR("While opening device! Error: " .. err)
        else
          posix.tcsetattr(file_descriptor, 0, {
            cflag = posix.B115200 + posix.CS8 + posix.CSIZE + posix.PARENB + posix.OPOST,
            iflag = posix.IGNBRK + posix.BRKINT + posix.PARMRK + posix.ISTRIP + posix.IGNCR + posix.IXON,
            oflag = posix.OPOST,
            cc = {
              [posix.VTIME] = 0,
              [posix.VMIN] = 1
            }
          })
          device = v
          break
        end
      end
      if file_descriptor == -1 then
        ros.ERROR("Could not open file or device for reading and writing")
      end
    end
  else
    ros.ERROR("No device could be found!!!")
    --print("No device could be found")
  end
end

-- This functions writes the data stored in the write buffer to the file/device
local function write()
  while true do
    if file_descriptor == nil or file_descriptor == -1 then
      init_file_descriptor(device, false)
    end
    if #write_buffer == 0 then
      ros.DEBUG("WRITER: nothing to write yield now")
      coroutine.yield()
    else
      for i,v in ipairs(write_buffer) do
        local bytes_written, err  = posix.write(file_descriptor, v)
        if err ~= nil then
          ros.ERROR("Writing data failed! Error: " ..  err)
          init_file_descriptor(device, true)
        end
      end
      write_buffer = {}
      ros.DEBUG("WRITER: job done yield now")
      coroutine.yield()
    end
  end
end

-- TODO Ggf. übersichtlicher gestalten

-- This function reads from the file/device and stores the data line by line into the read_buffer
local function read()
  local chunk_size = 4096
  local unfinished_line_buffer = nil

  while true do
    if file_descriptor == nil or file_descriptor == -1 then
      init_file_descriptor(device, false)
    end
    
    local buffer, err = posix.read(file_descriptor, chunk_size)

    if err ~= nil then
      -- Force reopen when error is not 11 (Resource temporarily unavailable) which is the usual case in non blocking mode when no data is available
      if err ~= "Resource temporarily unavailable" then
        ros.ERROR("Could not read data! " ..  err)
        init_file_descriptor(device, true)
      else
        ros.DEBUG("No data available to read! " ..  err)
      end
    end
    
    if buffer ~= nil and string.len(buffer) > 0 then

      -- Does read data contains any newline
      local data_contains_any_linebreak = string.match(buffer,"\n")
      -- Is the last read byte a newline
      local last_byte_is_new_line = string.match(string.sub(buffer, string.len(buffer), string.len(buffer)),"\n")
      local first_byte_is_new_line = string.match(string.sub(buffer, 1, 1),"\n")
      local all_lines_complete = false
      if data_contains_any_linebreak ~= nil and last_byte_is_new_line ~= nil then
        all_lines_complete = true
      end

      for line in string.gmatch(buffer,'[^\n]+') do
        -- When there is an incomplete line in local buffer
        if unfinished_line_buffer ~= nil then
          -- When first byte is not "\n" concat strings and write into read_buffer
          if first_byte_is_new_line == nil then
            read_buffer[#read_buffer+1] = unfinished_line_buffer .. line
            if data_contains_any_linebreak ~= nil then
              -- clear local buffer in case unfinished line has been completed
              unfinished_line_buffer = nil
            end
            -- When first byte is "\n" the inclomplete line is finished and written into the read_buffer and also the current line is written
          else
            read_buffer[#read_buffer+1] = unfinished_line_buffer
            read_buffer[#read_buffer+1] = line
            unfinished_line_buffer = nil
          end
        else
          read_buffer[#read_buffer+1] = line
        end
      end

      -- Only happens when the for loop above is skipped due to that the only byte is "\n"
      if string.len(buffer) == 1 and buffer == "\n" and unfinished_line_buffer ~= nil then
        read_buffer[#read_buffer+1] = unfinished_line_buffer
        unfinished_line_buffer = nil
      end

      -- If last line is incomplete remove it from read buffer and store in local before for later completion
      if not all_lines_complete then
        unfinished_line_buffer = read_buffer[#read_buffer]
        table.remove(read_buffer,#read_buffer)
      end

      ros.DEBUG("READER: job done yield now")
      -- coroutine.yield() -- DISCUSSION: Should the function read all data or yield after each chunck?
    else
      ros.DEBUG("READER: job unfinished yield now")
      coroutine.yield()
    end
  end
end


-- This function initiates the creation of the different messages
local function message_creation()
  local xamla_gripper_worker = coroutine.create(build_xamla_message)
  local xamla_ft_worker = coroutine.create(build_xamla_message)
  local xamla_imu_worker = coroutine.create(build_xamla_message)

  while true do
    coroutine.resume(xamla_gripper_worker, GRIPPER)
    coroutine.resume(xamla_ft_worker, FORCE_TORQUE)
    coroutine.resume(xamla_imu_worker, IMU)

    ros.DEBUG("MESSAGE CREATION: resumed all worker yield now")
    coroutine.yield()

    -- Create new coroutine in case coroutine died
    if coroutine.status(xamla_gripper_worker) == "dead" then
      xamla_gripper_worker = coroutine.create(build_xamla_message)
    end

    if coroutine.status(xamla_ft_worker) == "dead" then
      xamla_ft_worker = coroutine.create(build_xamla_message)
    end

    if coroutine.status(xamla_imu_worker) == "dead" then
      xamla_imu_worker = coroutine.create(build_xamla_message)
    end
  end
end

local function run()
  local write_worker = coroutine.create(write)
  local reader_worker = coroutine.create(read)
  local message_worker = coroutine.create(message_creation)
  init_file_descriptor(device, false)

  while true do
    if not ros.ok() then
      return
    end

    if not service_queue:isEmpty() then
      ros.DEBUG('[!] incoming service call')
      service_queue:callAvailable()
    end

    if publisher_gripper:getNumSubscribers() == 0 then
      ros.DEBUG('gripper waiting for subscriber')
    else
      enqueue(GRIPPER_LEFT_FINGER_FORCE, nil)
      enqueue(GRIPPER_RIGHT_FINGER_FORCE, nil)
      enqueue(GRIPPER_POS_FB, nil)
      enqueue(GRIPPER_GRIP_FORCE, nil)
    end

    if publisher_force_torque:getNumSubscribers() == 0 then -- TODO: Wenn keine Nachricht gesendet wird, dann wird nicht unsubscribed?
      ros.DEBUG('force torque waiting for subscriber')
    else
      enqueue(FT_X, nil)
      enqueue(FT_Y, nil)
      enqueue(FT_Z, nil)
      enqueue(FT_A, nil)
      enqueue(FT_B, nil)
      enqueue(FT_C, nil)
    end

    if publisher_imu:getNumSubscribers() == 0 then
      ros.DEBUG('imu waiting for subscriber')
    else
      enqueue(IMU_A, nil)
      enqueue(IMU_B, nil)
      enqueue(IMU_C, nil)
      enqueue(IMU_DA, nil)
      enqueue(IMU_DB, nil)
      enqueue(IMU_DC, nil)
    end

    coroutine.resume(write_worker)
    coroutine.resume(reader_worker)
    coroutine.resume(message_worker)

    if debug then
      for i,v in ipairs(read_buffer) do
        print(v)
      end
      print("-----------------------------")
    end

    -- Clear the read buffer
    read_buffer = {}

    if coroutine.status(write_worker) == "dead" then
      write_worker = coroutine.create(write)
    end
    if coroutine.status(reader_worker) == "dead" then
      reader_worker = coroutine.create(read)
    end
    if coroutine.status(message_worker) == "dead" then
      message_worker = coroutine.create(message_creation)
    end

    sys.sleep(0.1)
    ros.spinOnce()
  end
end

init()
run()
ros.shutdown()
