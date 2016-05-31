local ros = require "ros"

local path = require "pl.path"

local posix = require "posix"

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
local sendConfigToBoard

-- Gripper --
local gripper_spec
local publisher_gripper
local service
local commands_for_gripper = {}
local send_set_command_spec

local min_gripper_in_m = 0.0
local max_gripper_in_m = 0.087

local publisher_command_return
local command_return_spec
local call_sequence = 1

-- Force Torque --
local publisher_force_torque
local force_torque_spec
local commands_for_force_torque = {}

-- IMU --
local imu_spec
local publisher_imu
local commands_for_imu = {}

-- Joint State Gripper --
local joint_state_spec
local publisher_joint_state
local joint_state_time

-- Identificaton constants
local GRIPPER = 1
local FORCE_TORQUE = 2
local IMU = 3

local file_descriptor = nil

-- Debug
local debug = false
local joint_state = 0

local is_reconnected = false -- Is used in init() function and in run()

-- Buffer for data which are supposed to be written to file/device
local write_buffer = {}
local write_buffer_max_lenght = 1024
-- Buffer for data which have been read from the file/device
local read_buffer = {}

local device = "/dev/ttyACM0"

local types_string = {"GRIPPER", "FORCE_TORQUE", "IMU"}

local char_byte_table = {
  ["\n"] = string.byte("\n"),
  ["K"] = string.byte("K"),
  ["O"] = string.byte("O"),
  ["("] = string.byte("("),
  [")"] = string.byte(")")
}

local config_prefix = "term0.wave"

-- defines the position of the data fiel in the process data
local process_data_position = {
  [GRIPPER_POS_FB] = 0,
  [GRIPPER_GRIP_FORCE] = 1,
  [GRIPPER_LEFT_FINGER_FORCE] = 2,
  [GRIPPER_RIGHT_FINGER_FORCE] = 3,
  [FT_X] = 4,
  [FT_Y] = 5,
  [FT_Z] = 6,
  [FT_A] = 7,
  [FT_B] = 8,
  [FT_C] = 9,
  [IMU_A] = 10,
  [IMU_B] = 11,
  [IMU_C] = 12,
  [IMU_DA] = 13,
  [IMU_DB] = 14,
  [IMU_DC] = 15
}

-- Hint: order is changed based on indicies. The order of the returned process data is defined by the numbers 0-15
local config_for_process_data = {
  [GRIPPER_POS_FB] = config_prefix .. process_data_position[GRIPPER_POS_FB] .. " = " .. GRIPPER_POS_FB .. "\n",
  [GRIPPER_GRIP_FORCE] = config_prefix .. process_data_position[GRIPPER_GRIP_FORCE] .. " = " .. GRIPPER_GRIP_FORCE .. "\n",
  [GRIPPER_LEFT_FINGER_FORCE] = config_prefix .. process_data_position[GRIPPER_LEFT_FINGER_FORCE] .. " = " .. GRIPPER_LEFT_FINGER_FORCE .. "\n",
  [GRIPPER_RIGHT_FINGER_FORCE] = config_prefix .. process_data_position[GRIPPER_RIGHT_FINGER_FORCE] .. " = " .. GRIPPER_RIGHT_FINGER_FORCE .. "\n",
  [FT_X] = config_prefix .. process_data_position[FT_X] .. " = " .. FT_X .. "\n",
  [FT_Y] = config_prefix .. process_data_position[FT_Y] .. " = " .. FT_Y .. "\n",
  [FT_Z] = config_prefix .. process_data_position[FT_Z] .. " = " .. FT_Z .. "\n",
  [FT_A] = config_prefix .. process_data_position[FT_A] .. " = " .. FT_A .. "\n",
  [FT_B] = config_prefix .. process_data_position[FT_B] .. " = " .. FT_B .. "\n",
  [FT_C] = config_prefix .. process_data_position[FT_C] .. " = " .. FT_C .. "\n",
  [IMU_A] = config_prefix .. process_data_position[IMU_A] .. " = " .. IMU_A .. "\n",
  [IMU_B] = config_prefix .. process_data_position[IMU_B] .. " = " .. IMU_B .. "\n",
  [IMU_C] = config_prefix .. process_data_position[IMU_C] .. " = " .. IMU_C .. "\n",
  [IMU_DA] = config_prefix .. process_data_position[IMU_DA] .. " = " .. IMU_DA .. "\n",
  [IMU_DB] = config_prefix .. process_data_position[IMU_DB] .. " = " .. IMU_DB .. "\n",
  [IMU_DC] = config_prefix .. process_data_position[IMU_DC] .. " = " .. IMU_DC .. "\n"
}

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

local function convertPosFbFromByteToMeter(value)
  return 0.087 / (3.0 - 230.0) * (value - 230.0) -- TODO: Has to be validated
end

local function convertPosFbFromMeterToByte(value)
  return math.floor((3.0-230.0)/0.087 * value + 230.0) -- TODO: Has to be validated
end

-- This function searches for devices in order to get possible paths for read/write
-- Input:
--  id_vendor - The vendor id of the desired device
--  id_prdocut - The product id of the desired device
-- Output:
--  found - Table with all connected devices (e.g. {"/dev/ttyACM0", "/dev/ttyACM1"}
local function findDevice(id_vendor, id_product)
  local function read_first_line(fn)
    local file = io.open(fn, 'rb')
    local ln
    if file ~= nil then
      ln = file:read('*l')
      file:close()
    end
    return ln
  end

  local function starts_with(string, start)
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
local function getEgomoDevices()
  local vendor_id = "0483"
  local product_id = "5740"

  return findDevice(vendor_id, product_id)
end

-- This function finds the corresponding command for the service
-- Input:
--  command - Command which was send from service caller (e.g. "reset")
--  command_list - List containg commands for search
-- Ouput:
--  v - The corresponding command (e.g. "gripper0.reset")
local function chooseCommand(command, command_list)
  for i,v in ipairs(command_list) do
    if string.find(v,command) then
      return v
    end
  end
  return nil
end

-- Handler for service SendSetCommand
local function sendSetCommandHandler(request, response, header)
  if request.command_name == "joint_state" then
    joint_state = math.rad(request.value)
    return true
  end

  local command_to_send = chooseCommand(request.command_name, commands_for_gripper)

  if command_to_send ~= nil then
    if request.command_name == "pos_cmd" then
      local converted_value = convertPosFbFromMeterToByte(request.value)
      local min_pos_value = 0
      local max_pos_value = 255
      if converted_value < min_pos_value then
        converted_value = min_pos_value
      elseif converted_value > max_pos_value then
        converted_value = max_pos_value
      end
      ros.DEBUG("Value in m " .. request.value .. " was converted to byte: " .. converted_value)
      enqueue(command_to_send, converted_value)
    else
      enqueue(command_to_send, request.value)
    end
    response.response = "command accepted"
  else
    response.response = "command not valid"
    return false
  end
  return true
end

-- Function in order to do initialization used components (e.g. Services)
local function init()

  if not is_reconnected then
    ros.init('xamla_egomo')
  else
    publisher_gripper:shutdown()
    publisher_force_torque:shutdown()
    publisher_imu:shutdown()
    publisher_joint_state:shutdown()
    publisher_command_return:shutdown()
    service:shutdown()
    nodehandle:shutdown()
    spinner:stop()
    ros.init('xamla_egomo')
    print("Node reconnected to master...")
  end

  spinner = ros.AsyncSpinner()
  spinner:start()

  nodehandle = ros.NodeHandle()

  -- Gripper --
  add_commands_for_gripper()
  gripper_spec = ros.MsgSpec('egomo_msgs/XamlaGripper')
  publisher_gripper = nodehandle:advertise("XamlaGripper", gripper_spec)

  -- Experimental
  command_return_spec = ros.MsgSpec('std_msgs/Header')
  publisher_command_return = nodehandle:advertise("XamlaGripperCommandReturn", command_return_spec)

  send_set_command_spec = ros.SrvSpec('egomo_msgs/SendGripperSetCommand')
  service = nodehandle:advertiseService('egomo_msgs/SendGripperSetCommand', send_set_command_spec, sendSetCommandHandler)

  -- Force Torque --
  add_commands_for_force_torque()
  force_torque_spec = ros.MsgSpec('geometry_msgs/WrenchStamped')
  publisher_force_torque = nodehandle:advertise("XamlaForceTorque", force_torque_spec)

  -- IMU --
  add_commands_for_imu()
  imu_spec = ros.MsgSpec('geometry_msgs/AccelStamped')
  publisher_imu = nodehandle:advertise("XamlaIOIMU", imu_spec)

  -- Joint State Gripper --
  joint_state_spec = ros.MsgSpec('sensor_msgs/JointState')
  publisher_joint_state = nodehandle:advertise("joint_states", joint_state_spec)
end

local function assignValuesToJointStateMessage(joint_state_message, pos_fb)
  --local previous_position
  local value_in_rad = 0.8 - ((0.8/0.085) * convertPosFbFromByteToMeter(pos_fb))

  if value_in_rad < 0 then
    value_in_rad = 0
  elseif value_in_rad > 0.8 then
    value_in_rad = 0.8
  end

  --joint_state_message.position:set(torch.DoubleTensor({value_in_rad}))
  -- TODO: Bisher nur Debug
  joint_state_message.position:set(torch.DoubleTensor({joint_state}))
end

-- This function assigns the a value to a field of a message (if possible)
-- Input:
--  message - The message the value is supposed to be assigned to
--  field_name - The name of the field to which the value is supposed to be assigned to
--  value - The value which is supposed to be assigned
--  type - The type of the message (GRIPPER = 1, FORCETORQUE = 2, IMU = 3)
-- Output:
--  true if value was assigned; false otherwise
local function assignValuesToMessage(message, field_name, value, type)
  if type == GRIPPER then
    if field_name == GRIPPER_GRIP_FORCE then
      message.grip_force = value
      return true
    elseif field_name == GRIPPER_POS_FB then
      if message.spec.type == "sensor_msgs/JointState" then
        assignValuesToJointStateMessage(message,value)
        return true
      else
        local current_position = convertPosFbFromByteToMeter(value)
        if current_position < min_gripper_in_m then
          current_position = min_gripper_in_m
        elseif current_position > max_gripper_in_m then
          current_position = max_gripper_in_m
        end
        message.pos_fb = math.abs(current_position)
        return true
      end
    elseif field_name == GRIPPER_LEFT_FINGER_FORCE then
      message.left_finger_force = value
      return true
    elseif field_name == GRIPPER_RIGHT_FINGER_FORCE then
      message.right_finger_force = value
      return true
    else
      return false
    end
  elseif type == FORCE_TORQUE then
    if field_name == FT_X then
      message.wrench.force.x = value
      return true
    elseif field_name == FT_Y then
      message.wrench.force.y = value
      return true
    elseif field_name == FT_Z then
      message.wrench.force.z = value
      return true
    elseif field_name == FT_A then
      message.wrench.torque.x = value
      return true
    elseif field_name == FT_B then
      message.wrench.torque.y = value
      return true
    elseif field_name == FT_C then
      message.wrench.torque.z = value
      return true
    else
      return false
    end
  elseif type == IMU then
    if field_name == IMU_A then
      message.accel.linear.x = value
      return true
    elseif field_name == IMU_B then
      message.accel.linear.y = value
      return true
    elseif field_name == IMU_C then
      message.accel.linear.z = value
      return true
    elseif field_name == IMU_DA then
      message.accel.angular.x = value
      return true
    elseif field_name == IMU_DB then
      message.accel.angular.y = value
      return true
    elseif field_name == IMU_DC then
      message.accel.angular.z = value
      return true
    else
      return false
    end
  end
  return false
end

local function hasValue(table, value)
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
--  field_name - Name of the field corresponds to this value (e.g. "gripper0.pos_fb")
--  value - The corresponding value as a number (e.g. 1.004)
local function parseData(line, commands, blacklist)
  local field_name
  local value

  for i,v in ipairs(commands) do
    local first, last = string.find(line, v .. " = ")
    if first ~= nil then
      if hasValue(blacklist, v) then
        return nil
      else
        field_name = v
        value = string.sub(line,last,string.len(line))
        return field_name, tonumber(value)
      end
    end
  end
  return nil
end

-- This function is used to parse a read line. Returns nil if the
-- line does not contain a defined command
-- Input:
--  line - String which will be used to find a specific known command (e.g. "gripper0.pos_fb <= gripper0.pos_fb = 1.004000")
-- Output:
--  values - The corresponding value as a number (e.g. 1.004)
local function parseProcessData(line)
  local values = {}

  for value in string.gmatch(string.sub(line,2,-2),'[^,]+') do
    values[#values+1] = tonumber(value)
  end

  return values
end

-- This function is responsible for creating and publishing the messages based on the read data (single line protocol).
-- Notice: only one message is sent even when enough data is received for more messages. Due to blacklist concept values which
-- have been set already won't be replace by newer values. This might lead to that some values will be discarded. Use process data
-- concept to prevent discarding values.
-- Input:
--  type - Defines the type of the message (e.g. GRIPPER, FORCE_TORQUE, IMU)
local function buildXamlaMessage(type)
  local message
  local MESSAGE_PARAMETERS
  local used_command_list
  local used_publisher
  local number_of_set_parameters = 0
  local blacklist = {}

  -- Initialize components based on type
  if type == GRIPPER then
    message = ros.Message('egomo_msgs/XamlaGripper')
    MESSAGE_PARAMETERS = 4
    used_command_list = commands_for_gripper
    used_publisher = publisher_gripper
  elseif type == FORCE_TORQUE then
    message = ros.Message('geometry_msgs/WrenchStamped')
    MESSAGE_PARAMETERS = 6
    used_command_list = commands_for_force_torque
    used_publisher = publisher_force_torque
  elseif type == IMU then
    message = ros.Message('geometry_msgs/AccelStamped')
    MESSAGE_PARAMETERS = 6
    used_command_list = commands_for_imu
    used_publisher = publisher_imu
  end

  message.header.frame_id = "ee_link"

  while true do
    for i,line in ipairs(read_buffer) do
      -- Skip line if it is answer of an previous set command
      if string.byte(line,1) ~= char_byte_table["O"] and string.byte(line,2) ~= char_byte_table["K"] then
        -- Inspect the line and look for corresponding field and value
        local field_name, value = parseData(line, used_command_list, blacklist)
        if field_name ~= nil and value ~= nil then
          -- Try to assign the value to the given field
          if assignValuesToMessage(message, field_name, value, type) then
            table.insert(blacklist, field_name)
            number_of_set_parameters = number_of_set_parameters + 1;
          end
        end
      else
        local return_message = ros.Message('std_msgs/Header')
        return_message.seq = call_sequence
        call_sequence = call_sequence + 1
        return_message.stamp = ros.Time.now()
        return_message.frame_id = line
        publisher_command_return:publish(return_message)
      end
    end

    -- If message is complete publish message
    if number_of_set_parameters == MESSAGE_PARAMETERS then
      message.header.stamp = ros.Time.now()
      used_publisher:publish(message)
      blacklist = {}
      number_of_set_parameters = 0
      ros.DEBUG("XAMLA_" .. types_string[type] .. ": Message send yield now")
      coroutine.yield()
    else
      ros.DEBUG("XAMLA_" .. types_string[type] .. ": Message unfinished yield now")
      coroutine.yield()
    end
  end
end

-- This function checks whether the input is a valid command and returns the type

-- Input:
--  command - Command which is used for comparision
-- Output:
--  type - The type to which the command belongs to (Gripper, FT, IMU)

local function getType(command)
  for i,v in ipairs(commands_for_gripper) do
    if command == v then
      return GRIPPER
    end
  end
  for i,v in ipairs(commands_for_force_torque) do
    if command == v then
      return FORCE_TORQUE
    end
  end
  for i,v in ipairs(commands_for_imu) do
    if command == v then
      return FORCE_TORQUE
    end
  end
  return nil
end

-- This function is responsible for creating and publishing the messages based on the read data
local function buildXamlaMessageBasedOnProcessData()
  local message_gripper
  local message_force_torque
  local message_imu
  local message_join_state
  local type

  message_gripper = ros.Message('egomo_msgs/XamlaGripper')
  message_force_torque = ros.Message('geometry_msgs/WrenchStamped')
  message_imu = ros.Message('geometry_msgs/AccelStamped')
  message_join_state = ros.Message('sensor_msgs/JointState')

  message_gripper.header.frame_id = "ee_link"
  message_force_torque.header.frame_id = "ee_link"
  message_imu.header.frame_id = "ee_link"
  message_join_state.header.frame_id = "ee_link"
  message_join_state.name = {"robotiq_85_finger_right_1_joint"}

  while true do
    for i,line in ipairs(read_buffer) do
      -- Only process line if it is process data
      if string.byte(line,1) == char_byte_table["("] and string.byte(line,-1) == char_byte_table[")"] then
        -- Inspect the line and look for corresponding field and value
        local values = parseProcessData(line)
        if #values ~= 0 then
          for i, v in pairs(config_for_process_data) do
            type = getType(i)
            if type == GRIPPER then
              -- process_data_position[i]+1 --> the io-board starts counting at 0 therefore index is incremented by 1
              assignValuesToMessage(message_gripper, i, values[process_data_position[i]+1], type)
              -- Special case for the joint state of the gripper
              if i == GRIPPER_POS_FB then
                assignValuesToMessage(message_join_state, i, values[process_data_position[i]+1], type)
              end
            elseif type == FORCE_TORQUE then
              assignValuesToMessage(message_force_torque, i, values[process_data_position[i]+1], type)
            elseif type ==  IMU then
              assignValuesToMessage(message_imu, i, values[process_data_position[i]+1], type)
            end
          end
        end
      else
        local return_message = ros.Message('std_msgs/Header')
        return_message.seq = call_sequence
        call_sequence = call_sequence + 1
        return_message.stamp = ros.Time.now()
        return_message.frame_id = line
        publisher_command_return:publish(return_message)
      end
      message_gripper.header.stamp = ros.Time.now()
      publisher_gripper:publish(message_gripper)
      message_force_torque.header.stamp = ros.Time.now()
      publisher_force_torque:publish(message_force_torque)
      message_imu.header.stamp = ros.Time.now()
      publisher_imu:publish(message_imu)
      message_join_state.header.stamp = ros.Time.now()
      publisher_joint_state:publish(message_join_state)
    end
    ros.DEBUG("XAMLA_PUBLISHER: Messages send yield now")
    coroutine.yield()
  end
end

-- This function adds some signs to the command in order to meet the defined communication protocol
local function buildCompleteCommand(command, value)
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
  local complete_msg = buildCompleteCommand(command, value)
  table.insert(write_buffer, complete_msg)
end

-- This function opens the file/device for reading and writing
-- Input:
--  device - path or device (e.g. "/dev/ttyACM0")
local function initFileDescriptor(device, error_on_read_write)

  -- Scans for devices of type egomo IO board
  local devices = getEgomoDevices()

  if #devices ~= 0 then
    if file_descriptor == nil or file_descriptor == -1 or error_on_read_write then
      if file_descriptor ~= nil and file_descriptor ~= -1 then
        posix.close(file_descriptor)
        file_descriptor = nil
      end
      -- Run through all devices and try to open it
      for i, v in ipairs(devices) do
        local err
        file_descriptor, err = posix.open(v, posix.O_RDWR + posix.O_NONBLOCK);
        -- Set device if open was successful and break loop
        if err ~= nil then
          ros.ERROR("While opening device! Error: " .. err)
        else
          posix.tcsetattr(file_descriptor, 0, {
            cflag = posix.B115200 + posix.CS8 + posix.HUPCL + posix.CREAD,
            iflag = posix.IGNPAR,
            cc = {
              [posix.VTIME] = 0,
              [posix.VMIN] = 1
            }
          })
          device = v
          sendConfigToBoard()
          break
        end
      end
      if file_descriptor == nil or file_descriptor == -1 then
        ros.ERROR("Could not open file or device for reading and writing")
      end
    end
  else
    ros.ERROR("No device could be found!!!")
  end
end

-- This function sends the current config for the process data to the board
function sendConfigToBoard()
  if file_descriptor == nil or file_descriptor == -1 then
    initFileDescriptor(device, false)
  end

  for i,v in pairs(config_for_process_data) do
    local bytes_written, err  = posix.write(file_descriptor, v)

    if err ~= nil then
      if err ~= "Resource temporarily unavailable" then
        ros.ERROR("Could not write data! " ..  err)
        initFileDescriptor(device, true)
      else
        ros.DEBUG("Could not write data! " ..  err)
      end
    end
  end
end

-- This functions writes the data stored in the write buffer in one message to the file/device
local function writeMessageAtOnce()
  while true do
    if file_descriptor == nil or file_descriptor == -1 then
      initFileDescriptor(device, false)
    end
    if #write_buffer == 0 then
      ros.DEBUG("WRITER: nothing to write yield now")
      coroutine.yield()
    else
      local not_written_elements = {}

      local message = ""
      for i,v in ipairs(write_buffer) do
        message = message .. v
      end

      local bytes_written, err  = posix.write(file_descriptor, v)

      if err ~= nil then
        not_written_elements[#not_written_elements+1] = v
        if err ~= "Resource temporarily unavailable" then
          ros.ERROR("Could not write data! " ..  err)
          initFileDescriptor(device, true)
        else
          ros.INFO("Could not write data! " ..  err)
        end
      end
      write_buffer = not_written_elements
    end

    if #write_buffer > write_buffer_max_lenght then
      write_buffer = {}
      ros.ERROR("WRITER: Write buffer has more than " .. write_buffer_max_lenght .. " elements and was cleared now")
    end
    ros.DEBUG("WRITER: job done yield now")
    coroutine.yield()
  end
end

-- This functions writes the data stored in the write buffer line by line to the file/device
local function write()
  while true do
    if file_descriptor == nil or file_descriptor == -1 then
      initFileDescriptor(device, false)
    end
    if #write_buffer == 0 then
      ros.DEBUG("WRITER: nothing to write yield now")
      coroutine.yield()
    else
      local not_written_elements = {}

      for i,v in ipairs(write_buffer) do
        local bytes_written, err  = posix.write(file_descriptor, v)

        if err ~= nil then
          not_written_elements[#not_written_elements+1] = v
          if err ~= "Resource temporarily unavailable" then
            ros.ERROR("Could not write data! " ..  err)
            initFileDescriptor(device, true)
          else
            ros.INFO("Could not write data! " ..  err)
          end
        end
      end

      write_buffer = not_written_elements
      if #write_buffer > write_buffer_max_lenght then
        write_buffer = {}
        ros.ERROR("WRITER: Write buffer has more than " .. write_buffer_max_lenght .. " elements and was cleared now")
      end
      ros.DEBUG("WRITER: job done yield now")
      coroutine.yield()
    end
  end
end

-- This function reads from the file/device and stores the data line by line into the read_buffer
local function read()
  local chunk_size = 4096
  local unfinished_line_buffer = nil

  while true do
    if file_descriptor == nil or file_descriptor == -1 then
      initFileDescriptor(device, false)
    end

    local buffer, err = posix.read(file_descriptor, chunk_size)

    if err ~= nil then
      -- Force reopen when error is not 11 (Resource temporarily unavailable) which is the usual case in non blocking mode when no data is available
      if err ~= "Resource temporarily unavailable" then
        ros.ERROR("Could not read data! " ..  err)
        initFileDescriptor(device, true)
      else
        ros.DEBUG("No data available to read! " ..  err)
      end
    end

    if buffer ~= nil and #buffer > 0 then

      -- Does read data contains any newline
      local data_contains_any_linebreak = string.match(buffer,"\n")
      -- Is the last read byte a newline
      local last_byte_is_new_line = buffer:byte(-1) == char_byte_table["\n"]
      local first_byte_is_new_line = buffer:byte(1) == char_byte_table["\n"]
      local all_lines_complete = false

      if data_contains_any_linebreak ~= nil and last_byte_is_new_line then
        all_lines_complete = true
      end

      for line in string.gmatch(buffer,'[^\n]+') do
        -- When there is an incomplete line in local buffer
        if unfinished_line_buffer ~= nil then
          -- When first byte is not "\n" concat strings and write into read_buffer
          if not first_byte_is_new_line then
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
      if #buffer == 1 and buffer:byte(1) == char_byte_table["\n"] and unfinished_line_buffer ~= nil then
        read_buffer[#read_buffer+1] = unfinished_line_buffer
        unfinished_line_buffer = nil
      end

      -- If last line is incomplete remove it from read buffer and store in local before for later completion
      if not all_lines_complete then
        unfinished_line_buffer = read_buffer[#read_buffer]
        table.remove(read_buffer,#read_buffer)
      end

      ros.DEBUG("READER: job done yield now")
      -- coroutine.yield() neccesarry when device is sending data continously
    else
      ros.DEBUG("READER: job unfinished yield now")
      coroutine.yield()
    end
  end
end

-- Checks if any subsriber exists
local function isAnySubscribed()
  if publisher_gripper:getNumSubscribers() > 0 or publisher_force_torque:getNumSubscribers() > 0 or publisher_imu:getNumSubscribers() > 0
    or publisher_joint_state:getNumSubscribers() > 0 or publisher_command_return:getNumSubscribers() > 0 then
    return true
  end
  return false
end

-- This function initiates the creation of the different messages based on the process data concept
local function messageCreationWithProcessData()
  local xamla_message_publisher = coroutine.create(buildXamlaMessageBasedOnProcessData)

  while true do
    if isAnySubscribed() then
      coroutine.resume(xamla_message_publisher)
    end

    ros.DEBUG("MESSAGE CREATION: resumed all worker yield now")
    coroutine.yield()

    -- Create new coroutine in case coroutine died
    if coroutine.status(xamla_message_publisher) == "dead" then
      ros.WARN("MESSAGE PUBLISHER died and will be reinitialized")
      xamla_message_publisher = coroutine.create(buildXamlaMessageBasedOnProcessData)
    end
  end
end

-- This function initiates the creation of the different messages
local function messageCreation()
  local xamla_gripper_worker = coroutine.create(buildXamlaMessage)
  local xamla_ft_worker = coroutine.create(buildXamlaMessage)
  local xamla_imu_worker = coroutine.create(buildXamlaMessage)

  while true do
    if publisher_gripper:getNumSubscribers() > 0 then
      coroutine.resume(xamla_gripper_worker, GRIPPER)
    end
    if publisher_force_torque:getNumSubscribers() > 0 then
      coroutine.resume(xamla_ft_worker, FORCE_TORQUE)
    end
    if publisher_imu:getNumSubscribers() > 0 then
      coroutine.resume(xamla_imu_worker, IMU)
    end

    ros.DEBUG("MESSAGE CREATION: resumed all worker yield now")
    coroutine.yield()

    -- Create new coroutine in case coroutine died
    if coroutine.status(xamla_gripper_worker) == "dead" then
      ros.WARN("GRIPPER WORKER died and will be reinitialized")
      xamla_gripper_worker = coroutine.create(buildXamlaMessage)
    end

    if coroutine.status(xamla_ft_worker) == "dead"  then
      ros.WARN("FORCE TORQUE WORKER died and will be reinitialized")
      xamla_ft_worker = coroutine.create(buildXamlaMessage)
    end

    if coroutine.status(xamla_imu_worker) == "dead"  then
      ros.WARN("IMU WORKER died and will be reinitialized")
      xamla_imu_worker = coroutine.create(buildXamlaMessage)
    end
  end
end

-- Main loop using process data protocoll
local function runUsingProcessData()
  local write_worker = coroutine.create(write)
  local reader_worker = coroutine.create(read)
  local message_worker = coroutine.create(messageCreationWithProcessData)
  initFileDescriptor(device, false)

  while true do
    if not ros.ok() then
      return
    end

    if not ros.master.check() then
      print("Master down trying to reconnect...")
      while not ros.master.check() do
        sys.sleep(0.5)
      end
      is_reconnected = true
      init()
    end

    if isAnySubscribed() then
      enqueue("print")
    end

    coroutine.resume(write_worker)
    coroutine.resume(reader_worker)

    if is_reconnected then
      message_worker = coroutine.create(messageCreationWithProcessData)
      is_reconnected = false
    end
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
      ros.WARN("WRITER WORKER died and will be reinitialized")
      write_worker = coroutine.create(write)
    end
    if coroutine.status(reader_worker) == "dead" then
      ros.WARN("READER WORKER died and will be reinitialized")
      reader_worker = coroutine.create(read)
    end
    if coroutine.status(message_worker) == "dead" then
      ros.WARN("MESSAGE WORKER died and will be reinitialized")
      message_worker = coroutine.create(messageCreationWithProcessData)
    end
    -- 0.005 is a good value; smaller values lead to that writing data can fail because device is temporarily unavailable
    -- May increase this value when more messages are added
    sys.sleep(0.005)
    ros.spinOnce()
  end
end

-- Main loop using the single line protocol
local function run()
  local write_worker = coroutine.create(write)
  local reader_worker = coroutine.create(read)
  local message_worker = coroutine.create(messageCreation)
  initFileDescriptor(device, false)

  while true do
    if not ros.ok() then
      return
    end

    if not ros.master.check() then
      print("Master down trying to reconnect...")
      while not ros.master.check() do
        sys.sleep(0.5)
      end
      is_reconnected = true
      init()
    end

    if publisher_gripper:getNumSubscribers() == 0 then
      ros.DEBUG('gripper waiting for subscriber')
    else
      enqueue(GRIPPER_POS_FB)
      enqueue(GRIPPER_GRIP_FORCE)
      enqueue(GRIPPER_LEFT_FINGER_FORCE)
      enqueue(GRIPPER_RIGHT_FINGER_FORCE)
    end

    if publisher_force_torque:getNumSubscribers() == 0 then -- TODO: Wenn keine Nachricht gesendet wird, dann wird nicht unsubscribed?
      ros.DEBUG('force torque waiting for subscriber')
    else
      enqueue(FT_X)
      enqueue(FT_Y)
      enqueue(FT_Z)
      enqueue(FT_A)
      enqueue(FT_B)
      enqueue(FT_C)
    end

    if publisher_imu:getNumSubscribers() == 0 then
      ros.DEBUG('imu waiting for subscriber')
    else
      enqueue(IMU_A)
      enqueue(IMU_B)
      enqueue(IMU_C)
      enqueue(IMU_DA)
      enqueue(IMU_DB)
      enqueue(IMU_DC)
    end

    coroutine.resume(write_worker)
    coroutine.resume(reader_worker)

    if is_reconnected then
      message_worker = coroutine.create(messageCreation)
      is_reconnected = false
    end
    coroutine.resume(message_worker)

    if debug then
      for i,v in ipairs(read_buffer) do
        print(v)
      end
      print("----------------------------")
    end

    -- Clear the read buffer
    read_buffer = {}

    if coroutine.status(write_worker) == "dead" then
      ros.WARN("WRITER WORKER died and will be reinitialized")
      write_worker = coroutine.create(write)
    end
    if coroutine.status(reader_worker) == "dead" then
      ros.WARN("READER WORKER died and will be reinitialized")
      reader_worker = coroutine.create(read)
    end
    if coroutine.status(message_worker) == "dead" then
      ros.WARN("MESSAGE WORKER died and will be reinitialized")
      message_worker = coroutine.create(messageCreation)
    end
    -- 0.005 is a good value; smaller values lead to that writing data can fail because device is temporarily unavailable
    -- May increase this value when more messages are added
    sys.sleep(0.005)
    ros.spinOnce()
  end
end

init()
runUsingProcessData()
-- run() -- use this function for the old protocol
ros.shutdown()
