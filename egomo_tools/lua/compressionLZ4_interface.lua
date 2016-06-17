local egomotools=require 'egomo-tools.env'
local reorderBits=egomotools.lib['reorderBits']
local restoreBitOrder=egomotools.lib['restoreBitOrder']
local compressLZ4=egomotools.lib['compressLZ4']
local decompressLZ4=egomotools.lib['decompressLZ4']

local CompressionLZ4 = {} 


function CompressionLZ4.reorderBits(inData)
   local outData = torch.ByteTensor()
   reorderBits(inData:cdata(), outData:cdata())
   return outData
end

function CompressionLZ4.restoreBitOrder(inData, outData)
   outData = outData or torch.ByteTensor()
   restoreBitOrder(inData:cdata(), outData:cdata())
   return outData
end

function CompressionLZ4.compressLZ4(inData, outData)
   return compressLZ4(inData:cdata(), outData:cdata())
end

function CompressionLZ4.decompressLZ4(inData, uncompressedSize)
   print("indata datatype:")
   print(inData:type())
   local outData = torch.ByteTensor()
   local outDataSize = decompressLZ4(inData:cdata(), outData:cdata(), uncompressedSize)
   return outData, outDataSize
end

return CompressionLZ4
