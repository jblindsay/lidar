require "./lidar/*"
require "io"

# Module for manipulating LiDAR (laser altimetry) data.
module Lidar
  # A 16-bit per channel (UInt16) red-green-blue (RGB) data structure, used for specifying point colouring.
  struct RGBData
    @red : UInt16
    @green : UInt16
    @blue : UInt16

    def initialize
      @red = 0_u16
      @green = 0_u16
      @blue = 0_u16
    end

    property red
    property green
    property blue

    def to_s
      return "red: #{@red}, green: #{@green}, blue: #{@blue}"
    end
  end

  # A struct for holding a LAS file's Variable Length Record (VLR).
  struct VLR
    property reserved
    property user_id
    property record_id
    property record_length_after_header
    property description
    property binary_data

    def initialize
      @reserved = 0_u16
      @user_id = ""
      @record_id = 0_u16
      @record_length_after_header = 0_u16
      @description = ""
      @binary_data = [] of UInt8
    end

    def to_s
      s = "\tReserved: #{@reserved}\n" +
        "\tUser ID: #{@user_id}\n" +
        "\tRecord ID: #{@record_id}\n" +
        "\tRecord Length: #{@record_length_after_header}\n" +
        "\tDescription: #{@description.strip}\n"
      if @record_id == 34735
        shorts = [] of UInt16
        i = 0
        while i < @binary_data.size
          shorts << (@binary_data[i, 2].to_unsafe as UInt16*).value
          i += 2
        end
        s += "\tData: #{shorts}"
      elsif @record_id == 34736
        doubles = [] of Float64
        i = 0
        while i < @binary_data.size
          doubles << (@binary_data[i, 8].to_unsafe as Float64*).value
          i += 8
        end
        s += "\tData: #{doubles}"
      else
        s += "\tData: #{@binary_data.map { |x| x.chr }.join.strip}"
      end
      return s
    end
  end

  # This struct represents the bit field contained within a LAS file's PointData.
  struct PointBitField
    def initialize
      @value = 0_u8
    end

    property value

    def return_number
      return @value & 7_u8
    end

    def number_of_returns
      return (@value >> 3_u8) & 7_u8
    end

    def scan_direction_flag
      return ((@value >> 6_u8) & 1_u8) == 1_u8
    end

    def edge_of_flightline
      return ((@value >> 7_u8) & 1_u8) == 1_u8
    end

    def to_s
      s = "{value: #{@value}, return: #{return_number}, returns: #{number_of_returns}," +
        " scan dir flag: #{scan_direction_flag}, flightline: #{edge_of_flightline}}"
      return s
    end
  end

  # The point classification enum used in the LAS specifications v. 1.3.
  enum ClassificationName : UInt8
    NeverClassified  =  0
    Unclassified     =  1
    Ground           =  2
    LowVegetation    =  3
    MedVegetation    =  4
    HighVegetation   =  5
    Building         =  6
    LowPoint         =  7
    ModelKeyPoint    =  8
    Water            =  9
    Reserved1        = 10
    Reserved2        = 11
    OverlappingPoint = 12
    Reserved3        = 13
    Reserved4        = 14
    Reserved5        = 15
    Reserved6        = 16
    Reserved7        = 17
    Reserved8        = 18
    Reserved9        = 19
    Reserved10       = 20
    Reserved11       = 21
    Reserved12       = 22
    Reserved13       = 23
    Reserved14       = 24
    Reserved15       = 25
    Reserved16       = 26
    Reserved17       = 27
    Reserved18       = 28
    Reserved19       = 29
    Reserved20       = 30
    Reserved21       = 31
  end

  # This struct represents the classification bit field contained within a LAS file's PointData.
  struct ClassificationBitField
    def initialize
      @value = 0_u8
    end

    property value

    def classification_name
      return ClassificationName.new(value & 7_u8)
    end

    def synthetic
      return ((value >> 5_u8) & 1_u8) == 1_u8
    end

    def keypoint
      return ((value >> 6_u8) & 1_u8) == 1_u8
    end

    def withheld
      return ((value >> 7_u8) & 1_u8) == 1_u8
    end

    def to_s
      s = "{value: #{@value}, name: #{classification_name}, synthetic: #{synthetic}," +
        " keypoint: #{keypoint}, withheld: #{withheld}}"
      return s
    end
  end

  # Simple data structure for storing XYZ triplets. Use with LasFile.get_xyz_data.
  struct XYZData
    def initialize
      @x = 0_f64
      @y = 0_f64
      @z = 0_f64
    end

    def initialize(@x : Float64, @y : Float64, @z : Float64)
    end

    property x
    property y
    property z

    def to_s
      return "x=#{sprintf("%.4f", @x)}, y=#{sprintf("%.4f", @y)}, z=#{sprintf("%.4f", @z)}"
    end
  end

  # Simple data structure for storing XYZ triplets. Use with LasFile.get_xyzi_data.
  struct XYZIData
    def initialize
      @x = 0_f64
      @y = 0_f64
      @z = 0_f64
      @intensity = 0_u16
    end

    def initialize(@x : Float64, @y : Float64, @z : Float64, @intensity : UInt16)
    end

    property intensity

    def to_s
      return ("x=#{sprintf("%.4f", @x)}, y=#{sprintf("%.4f", @y)}, z=#{sprintf("%.4f", @z)}, i=#{@intensity}")
    end
  end

  # Structure for storing the information regarding points (e.g. x, y, z, intensity,
  # classification, return data) from a LAS point record.
  struct PointData
    def initialize
      @x = 0_f64
      @y = 0_f64
      @z = 0_f64
      @intensity = 0_u16
      @bit_field = PointBitField.new
      @class_field = ClassificationBitField.new
      @scan_angle = 0_i8
      @user_data = 0_u8
      @point_source_id = 0_u16
    end

    property x
    property y
    property z
    property intensity
    property bit_field
    property class_field
    property scan_angle
    property user_data
    property point_source_id

    def to_s
      s = "x=#{sprintf("%.4f", @x)}, " +
        "y=#{sprintf("%.4f", @y)}, " +
        "z=#{sprintf("%.4f", @z)}, " +
        "intensity=#{@intensity}, " +
        "bit field=#{(@bit_field).to_s}, " +
        "class field=#{(@class_field).to_s}, " +
        "scan angle=#{@scan_angle}, " +
        "user data=#{@user_data}, " +
        "point source ID=#{@point_source_id}"
      return s
    end
  end

  # LAS file header structure
  struct LasHeader
    def initialize
      @file_signature = "LASF"
      @file_source_id = 0_u16
      @global_encoding = 0_u16
      @project_id1 = 0_u32
      @project_id2 = 0_u16
      @project_id3 = 0_u16
      @project_id4 = [] of UInt8 # ""
      @version_major = 0_u8
      @version_minor = 0_u8
      @system_identifier = "GAT by John Lindsay\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @generating_software = "Geospatial Analysis Tools\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @file_creation_day = 0_u16
      @file_creation_year = 0_u16
      @header_size = 0_u16
      @offset_to_points = 0_u32
      @number_of_vlrs = 0_u32
      @point_format = 0_u8
      @point_record_length = 0_u16
      @number_of_points = 0_u32
      @number_of_points_by_return = [] of UInt32
      @x_scale_factor = 0.01_f64
      @y_scale_factor = 0.01_f64
      @z_scale_factor = 0.001_f64
      @x_offset = 0_f64
      @y_offset = 0_f64
      @z_offset = 0_f64
      @max_x = -32768.0_f64 # Float64::MIN
      @min_x = 32768.0_f64  # Float64::MAX
      @max_y = -32768.0_f64
      @min_y = 32768.0_f64
      @max_z = -32768.0_f64
      @min_z = 32768.0_f64
      @waveform_data_start = 0_u64
    end

    property file_signature
    property file_source_id
    property global_encoding
    property project_id1
    property project_id2
    property project_id3
    property project_id4
    property version_major
    property version_minor
    property system_identifier
    property generating_software
    property file_creation_day
    property file_creation_year
    property header_size
    property offset_to_points
    property number_of_vlrs
    property point_format
    property point_record_length
    property number_of_points
    property number_of_points_by_return
    property x_scale_factor
    property y_scale_factor
    property z_scale_factor
    property x_offset
    property y_offset
    property z_offset
    property max_x
    property min_x
    property max_y
    property min_y
    property max_z
    property min_z
    property waveform_data_start

    def to_s
      s = "File Signature: #{@file_signature}\n" +
        "File Source ID: #{@file_source_id}\n" +
        "Global Encoding: #{@global_encoding}\n" +
        "Project ID1: #{@project_id1}\n" +
        "Project ID2: #{@project_id2}\n" +
        "Project ID3: #{@project_id3}\n" +
        "Project ID4: #{@project_id4}\n" +
        "Version: #{@version_major}.#{@version_minor}\n" +
        "System ID: #{@system_identifier.strip}\n" +
        "Generating Software: #{@generating_software.strip}\n" +
        "File Creation Day: #{@file_creation_day}\n" +
        "File Creation Year: #{@file_creation_year}\n" +
        "Header Size: #{@header_size}\n" +
        "Offset to Points: #{@offset_to_points}\n" +
        "Number of VLRs: #{@number_of_vlrs}\n" +
        "Point Format: #{@point_format}\n" +
        "Point Record Length: #{@point_record_length}\n" +
        "Num. of Points: #{@number_of_points}\n" +
        "Num. Points by Return: #{@number_of_points_by_return}\n" +
        "X Scale Factor: #{sprintf("%.4f", @x_scale_factor)}\n" +
        "Y Scale Factor: #{sprintf("%.4f", @y_scale_factor)}\n" +
        "Z Scale Factor: #{sprintf("%.4f", @z_scale_factor)}\n" +
        "X Offset: #{sprintf("%.4f", @x_offset)}\n" +
        "Y Offset: #{sprintf("%.4f", @y_offset)}\n" +
        "Z Offset: #{sprintf("%.4f", @z_offset)}\n" +
        "Min. X: #{sprintf("%.4f", @min_x)}\n" +
        "Max. X: #{sprintf("%.4f", @max_x)}\n" +
        "Min. Y: #{sprintf("%.4f", @min_y)}\n" +
        "Max. Y: #{sprintf("%.4f", @max_y)}\n" +
        "Min. Z: #{sprintf("%.4f", @min_z)}\n" +
        "Max. Z: #{sprintf("%.4f", @max_z)}"
      if @version_major == 1 && @version_minor >= 3
        s += "\nWaveform Data Start: #{@waveform_data_start}"
      end
      return s
    end
  end

  # The main class for dealing with a LAS file data structure.
  class LasFile
    # returns the file name and full path of this LasFile.
    getter file_name
    @file_mode = "r"
    @file_name : String
    @file_size = 0

    # Gets and sets the LasHeader associated with this LasFile.
    property header
    @header : LasHeader

    # Gets and sets the variable length record (VLR) array associated with this LasFile.
    property vlr_data
    @vlr_data = [] of VLR
    @point_data = [] of PointData
    @gps_time_data = [] of Float64
    @rgb_data = [] of RGBData

    # Creates a LasFile based on a *file_name*.
    # The *file_mode* parameter must equal "r" for read-only, or "w" for write.
    #
    # Example
    #
    # ```
    # file_name = "/path/to/file/data.las"
    # lf1 = Lidar::LasFile.new file_name, "r"
    # puts("#{lf1.to_s}") # => prints file header info
    # puts("#{lf1[100]}") # => prints the 100th point record
    #
    # new_file = "/path/to/file/newFile.las"
    # lf2 = Lidar::LasFile.new new_file, "w"
    # # Add the header
    # lf2.add_header(lf1.header)
    # # Add the VLRs to the file
    # lf1.vlr_data.each do |vlr|
    #   lf2.add_vlr(vlr)
    # end
    #
    # # Add the point data to the file
    # (0...lf1.header.number_of_points).each do |i|
    #   lf2.add_point_record(lf1[i])
    # end
    #
    # lf2.write
    # ```
    def initialize(file_name, file_mode = "r")
      @file_name = file_name
      @header = LasHeader.new
      if file_mode.downcase == "r"
        @file_mode = "r"
        read
      elsif file_mode.downcase == "w"
        @file_mode = "w"
      else
        raise "Unrecognized file mode. Please use either 'r' for read-mode or 'w' for write mode."
      end
    end

    # Creates a new LasFile for writing to disc based on a *file_name*, *header*, *vlr_data*, and *record_data*.
    # The *file_mode* parameter will be set to "w" for write-access.
    #
    # **file_name** = a String containing the name of the file.
    #
    # **header** = a LasHeader struct used to initialize the file properties.
    #
    # **vlr_data** = an array of *VLR* structs containing each of the variable length records.
    #
    # **record_data** = an array of *PointData* structs containing the point data.
    #
    # See also: methods *add_header*, *add_vlr*, and *add_point_record*, which can be used to add these
    # components individually and after initialization using the standard initialize method in "w" file_mode.
    def initialize(file_name : String, header : LasHeader, vlr_data : Array(VLR), record_data : Array(PointData))
      @file_name = file_name
      @file_mode = "w"
      @header = header
      @header.number_of_vlrs = 0_u32
      @header.number_of_points = 0_u32
      @header.version_major = 1_u8
      @header.version_minor = 3_u8
      # These must be set by the data
      @header.min_x = Float64::MAX
      @header.max_x = Float64::MIN
      @header.min_y = Float64::MAX
      @header.max_y = Float64::MIN
      @header.min_z = Float64::MAX
      @header.max_z = Float64::MIN
      @header.system_identifier = "GAT by John Lindsay\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @header.generating_software = "Geospatial Analysis Tools\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @header.number_of_points_by_return = [0_u32, 0_u32, 0_u32, 0_u32, 0_u32]
      @vlr_data = vlr_data
      write
    end

    # Sets the properties of the LAS file using a specified LasHeader struct.
    def add_header(header : LasHeader)
      if @file_mode == "r"
        raise "This file has been accessed in read-only mode."
      end
      @header = header
      @header.number_of_vlrs = 0_u32
      @header.number_of_points = 0_u32
      @header.version_major = 1_u8
      @header.version_minor = 3_u8
      # These must be set by the data
      @header.min_x = Float64::MAX
      @header.max_x = Float64::MIN
      @header.min_y = Float64::MAX
      @header.max_y = Float64::MIN
      @header.min_z = Float64::MAX
      @header.max_z = Float64::MIN
      @header.system_identifier = "GAT by John Lindsay\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @header.generating_software = "Geospatial Analysis Tools\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}"
      @header.number_of_points_by_return = [0_u32, 0_u32, 0_u32, 0_u32, 0_u32]
    end

    # Adds a variable length record (VLR) to this LAS file.
    def add_vlr(vlr : VLR)
      @vlr_data << vlr
      @header.number_of_vlrs += 1
    end

    # Adds a point record (either type 0, 1, 2, or 3) to this LAS file.
    def add_point_record(p : (PointRecord0 | PointRecord1 | PointRecord2 | PointRecord3))
      if @file_mode == "r"
        raise "This file has been accessed in read-only mode."
      end
      @point_data << p.point
      @header.min_x = p.point.x if p.point.x < @header.min_x
      @header.max_x = p.point.x if p.point.x > @header.max_x
      @header.min_y = p.point.y if p.point.y < @header.min_y
      @header.max_y = p.point.y if p.point.y > @header.max_y
      @header.min_z = p.point.z if p.point.z < @header.min_z
      @header.max_z = p.point.z if p.point.z > @header.max_z
      @header.number_of_points_by_return[p.point.bit_field.return_number - 1] += 1
      case @header.point_format
      when 1
        @gps_time_data << (p as PointRecord1).gps_time
      when 2
        @rgb_data << (p as PointRecord2).rgb_data
      when 3
        @gps_time_data << (p as PointRecord3).gps_time
        @rgb_data << (p as PointRecord3).rgb_data
      end

      @header.number_of_points += 1
    end

    # Returns a string containing the LAS version number of this file.
    def get_version
      return "#{@header.version_major}.#{@header.version_minor}"
    end

    # Returns the size of the LAS file in megabytes.
    def size
      return "File size: #{@file_size.to_f / 1_000_000.0}MB"
    end

    # Returns a string represenation of the LAS file properties.
    def to_s
      return @header.to_s
    end

    # Reads the header, variable length records (VLRs), and point data of the LAS file.
    def read
      if File.extname(@file_name).downcase != ".las"
        # if !file_name.downcase.ends_with?("las")
        raise "The file extension is not '.las', indicating that this is not a las file."
        return
      end

      @file_size = File.size(@file_name)
      buffer = Slice(UInt8).new(@file_size)
      File.open(@file_name, "rb") do |file|
        file.read_fully(buffer)
        file.close
      end

      # #######################
      # Read the header data #
      # #######################

      # get the file version
      # file.pos = 24
      # @header.version_major = file.read_byte as UInt8
      # @header.version_minor = file.read_byte as UInt8
      # file.pos = 0

      # file_size = file.size
      # buffer = uninitialized UInt8[227]
      # read_bytes = @file.read(buffer.to_slice) #buffer.to_slice)
      # @file.close
      # buffer = Slice(UInt8).new(file_size)
      # file.read_fully(buffer)
      # file.close

      @header.file_signature = buffer[0, 4].map { |x| x.chr }.join
      if @header.file_signature != "LASF"
        raise "The file signature must be equal to 'LASF'. This may not be a LAS file."
        return
      end
      @header.file_source_id = (buffer[4, 2].to_unsafe as UInt16*).value
      @header.global_encoding = (buffer[6, 2].to_unsafe as UInt16*).value
      @header.project_id1 = (buffer[8, 4].to_unsafe as UInt32*).value
      @header.project_id2 = (buffer[12, 2].to_unsafe as UInt16*).value
      @header.project_id3 = (buffer[14, 2].to_unsafe as UInt16*).value
      @header.project_id4 = buffer[16, 8].to_a
      # @header.project_id4 = buffer[16, 8].map { |x|
      #   x
      # }

      @header.version_major = buffer[24]
      @header.version_minor = buffer[25]

      @header.system_identifier = buffer[26, 32].map { |x|
        if x.chr != '\u{0}'
          x.chr
        else
          " "
        end
      }.join # .strip
      @header.generating_software = buffer[58, 32].map { |x|
        if x.chr != '\u{0}'
          x.chr
        else
          " "
        end
      }.join
      @header.file_creation_day = (buffer[90, 2].to_unsafe as UInt16*).value
      @header.file_creation_year = (buffer[92, 2].to_unsafe as UInt16*).value
      @header.header_size = (buffer[94, 2].to_unsafe as UInt16*).value.to_u16
      @header.offset_to_points = (buffer[96, 4].to_unsafe as UInt32*).value
      @header.number_of_vlrs = (buffer[100, 4].to_unsafe as UInt32*).value
      @header.point_format = (buffer[104, 1].to_unsafe as UInt8*).value
      @header.point_record_length = (buffer[105, 2].to_unsafe as UInt16*).value
      @header.number_of_points = (buffer[107, 4].to_unsafe as UInt32*).value

      num_returns = 5
      if @header.version_major == 1_u8 && @header.version_minor > 3_u8
        num_returns = 7
      end
      offset = 111
      # @number_of_points_by_return = [] of UInt32
      @header.number_of_points_by_return = Array.new(num_returns) { |i|
        (buffer[offset + i * 4, 4].to_unsafe as UInt32*).value
      }
      offset += 4 * num_returns
      @header.x_scale_factor = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.y_scale_factor = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.z_scale_factor = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.x_offset = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.y_offset = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.z_offset = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.max_x = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.min_x = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.max_y = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.min_y = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.max_z = (buffer[offset, 8].to_unsafe as Float64*).value
      offset += 8
      @header.min_z = (buffer[offset, 8].to_unsafe as Float64*).value

      if @header.version_major == 1_u8 && @header.version_minor == 3_u8
        offset += 8
        @header.waveform_data_start = (buffer[offset, 8].to_unsafe as UInt64*).value
      end

      # ####################
      # Read the VLR data #
      # ####################
      offset = @header.header_size
      num = (buffer[100, 4].to_unsafe as Int32*).value # Num VLRs; note, Array.new takes an Int32 and not a UInt32
      @vlr_data = Array.new(num) { |i|
        vlr = VLR.new
        vlr.reserved = (buffer[offset, 2].to_unsafe as UInt16*).value
        vlr.user_id = buffer[offset + 2, 16].map { |x|
          if x.chr != '\u{0}'
            x.chr
          else
            " "
          end
        }.join
        vlr.record_id = (buffer[offset + 18, 2].to_unsafe as UInt16*).value
        vlr.record_length_after_header = (buffer[offset + 20, 2].to_unsafe as UInt16*).value
        vlr.description = buffer[offset + 22, 32].map { |x|
          if x.chr != '\u{0}'
            x.chr
          else
            " "
          end
        }.join
        # Read the data
        vlr.binary_data = buffer[offset + 54, vlr.record_length_after_header].to_a

        offset += 54 + vlr.record_length_after_header
        vlr
      }

      # ######################
      # Read the point data #
      # ######################
      num_points = (buffer[107, 4].to_unsafe as Int32*).value # Num Points; note, Array.new takes an Int32 and not a UInt32
      @point_data = Array.new(num_points) { |i|
        offset = @header.offset_to_points + i * @header.point_record_length
        point = PointData.new
        point.x = (buffer[offset, 4].to_unsafe as Int32*).value * @header.x_scale_factor + @header.x_offset
        offset += 4
        point.y = (buffer[offset, 4].to_unsafe as Int32*).value * @header.y_scale_factor + @header.y_offset
        offset += 4
        point.z = (buffer[offset, 4].to_unsafe as Int32*).value * @header.z_scale_factor + @header.z_offset
        offset += 4
        point.intensity = (buffer[offset, 2].to_unsafe as UInt16*).value
        offset += 2
        point.bit_field.value = buffer[offset]
        offset += 1
        point.class_field.value = buffer[offset]
        offset += 1
        point.scan_angle = (buffer[offset, 1].to_unsafe as Int8*).value
        offset += 1
        point.user_data = (buffer[offset, 1].to_unsafe as UInt8*).value
        offset += 1
        point.point_source_id = (buffer[offset, 2].to_unsafe as UInt16*).value
        point
      }

      if @header.point_format == 1
        # read the gps data
        @gps_time_data = Array.new(num_points) { |i|
          offset = @header.offset_to_points + 20 + i * @header.point_record_length
          (buffer[offset, 8].to_unsafe as Float64*).value
        }
      elsif @header.point_format == 2
        # read the rgb data
        @rgb_data = Array.new(num_points) { |i|
          rgb = RGBData.new
          offset = @header.offset_to_points + 20 + i * @header.point_record_length
          rgb.red = (buffer[offset, 2].to_unsafe as UInt16*).value
          rgb.green = (buffer[offset + 2, 2].to_unsafe as UInt16*).value
          rgb.blue = (buffer[offset + 4, 2].to_unsafe as UInt16*).value
          rgb
        }
      elsif @header.point_format == 3
        # read the gps data
        @gps_time_data = Array.new(num_points) { |i|
          offset = @header.offset_to_points + 20 + i * @header.point_record_length
          (buffer[offset, 8].to_unsafe as Float64*).value
        }
        # read the rgb data
        @rgb_data = Array.new(num_points) { |i|
          rgb = RGBData.new
          offset = @header.offset_to_points + 28 + i * @header.point_record_length
          rgb.red = (buffer[offset, 2].to_unsafe as UInt16*).value
          rgb.green = (buffer[offset + 2, 2].to_unsafe as UInt16*).value
          rgb.blue = (buffer[offset + 4, 2].to_unsafe as UInt16*).value
          rgb
        }
      end
    end

    # Writes the header, variable length records (VLRs), and point data of the LAS file to disc.
    def write
      if @file_mode == "r"
        raise "This file has been accessed in read-only mode."
      end

      # If the file exists, delete it
      if File.exists?(@file_name)
        File.delete(@file_name)
      end

      @header.x_offset = (header.min_x / 1000.0).floor * 1000.0
      @header.y_offset = (header.min_y / 1000.0).floor * 1000.0
      @header.z_offset = header.min_z.floor

      # The output file will be a LAS version 1.3 file
      @header.version_major = 1_u8
      @header.version_minor = 3_u8
      @header.header_size = 235_u16

      # What is the day of year and year?
      time = Time.now
      @header.file_creation_day = time.day_of_year.to_u16
      @header.file_creation_year = time.year.to_u16

      # What is the point record length?
      case @header.point_format
      when 0
        @header.point_record_length = 20_u16
      when 1
        @header.point_record_length = 28_u16
      when 2
        @header.point_record_length = 26_u16
      when 3
        @header.point_record_length = 34_u16
      end

      # What will the size of the final file be?
      @file_size = @header.header_size.to_i32
      vlr_data.each do |vlr|
        @file_size += 54 + vlr.record_length_after_header
      end
      @header.offset_to_points = @file_size.to_u32
      @file_size += @header.number_of_points * @header.point_record_length

      # Declare a slice of u8 of the file_size
      buffer = Slice(UInt8).new(@file_size)

      offset = 0

      # ##############################################################################################
      # The following is a failed attempt to use a MemoryIO. Although it seemed to work, it was many
      # times slower than the adopted method, which just copied data directly into a slice(UInt8)
      # ##############################################################################################

      # ##################################
      # # Write the header to the buffer #
      # ##################################
      #
      # buff = MemoryIO.new(@file_size)
      # buff.write @header.file_signature.to_slice
      # buff.write_bytes @header.@file_source_id, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@global_encoding, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@project_id1, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@project_id2, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@project_id3, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@project_id3, IO::ByteFormat::LittleEndian
      # @header.project_id4.each { |b| buff.write_bytes b, IO::ByteFormat::LittleEndian}
      # buff.write_bytes @header.@version_major, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@version_minor, IO::ByteFormat::LittleEndian
      # buff.write (fixed_width_string(@header.system_identifier, 32).to_unsafe as UInt8*).to_slice(32)
      # buff.write (fixed_width_string(@header.generating_software, 32).to_unsafe as UInt8*).to_slice(32)
      # buff.write_bytes @header.@file_creation_day, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@file_creation_year, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@header_size.to_u16, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@offset_to_points.to_u32, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@number_of_vlrs.to_u32, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@point_format, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@point_record_length.to_u16, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.number_of_points.to_u32, IO::ByteFormat::LittleEndian
      # (0..4).each do |i|
      #   buff.write_bytes @header.number_of_points_by_return[i], IO::ByteFormat::LittleEndian
      # end
      # buff.write_bytes @header.@x_scale_factor, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@y_scale_factor, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@z_scale_factor, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@x_offset, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@y_offset, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@z_offset, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@max_x, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@min_x, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@max_y, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@min_y, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@max_z, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@min_z, IO::ByteFormat::LittleEndian
      # buff.write_bytes @header.@waveform_data_start, IO::ByteFormat::LittleEndian
      #
      # ################################
      # # Write the VLRs to the buffer #
      # ################################
      # @vlr_data.each do |vlr|
      #   buff.write_bytes vlr.@reserved, IO::ByteFormat::LittleEndian
      #   buff.write (fixed_width_string(vlr.@user_id, 16).to_unsafe as UInt8*).to_slice(16)
      #   buff.write_bytes vlr.@record_id, IO::ByteFormat::LittleEndian
      #   buff.write_bytes vlr.@record_length_after_header, IO::ByteFormat::LittleEndian
      #   buff.write (fixed_width_string(vlr.description, 32).to_unsafe as UInt8*).to_slice(32)
      #   vlr.binary_data.each do |b|
      #     buff.write_bytes b, IO::ByteFormat::LittleEndian
      #   end
      # end
      #
      # ########################
      # # Write the point data #
      # ########################
      # case @header.point_format
      # when 0
      #   @point_data.each do |p|
      #     x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
      #     y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
      #     z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
      #     buff.write_bytes x, IO::ByteFormat::LittleEndian
      #     buff.write_bytes y, IO::ByteFormat::LittleEndian
      #     buff.write_bytes z, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.intensity, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.bit_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.class_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.scan_angle.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.user_data.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.point_source_id, IO::ByteFormat::LittleEndian
      #   end
      # when 1
      #   i = 0
      #   @point_data.each do |p|
      #     x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
      #     y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
      #     z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
      #     buff.write_bytes x, IO::ByteFormat::LittleEndian
      #     buff.write_bytes y, IO::ByteFormat::LittleEndian
      #     buff.write_bytes z, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.intensity, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.bit_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.class_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.scan_angle.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.user_data.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.point_source_id, IO::ByteFormat::LittleEndian
      #     # Output the gps time
      #     buff.write_bytes @gps_time_data[i], IO::ByteFormat::LittleEndian
      #     i += 1
      #   end
      #
      # when 2
      #   i = 0
      #   @point_data.each do |p|
      #     x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
      #     y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
      #     z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
      #     buff.write_bytes x, IO::ByteFormat::LittleEndian
      #     buff.write_bytes y, IO::ByteFormat::LittleEndian
      #     buff.write_bytes z, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.intensity, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.bit_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.class_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.scan_angle.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.user_data.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.point_source_id, IO::ByteFormat::LittleEndian
      #     # Output the rgb data
      #     buff.write_bytes @rgb_data[i].red.to_u16, IO::ByteFormat::LittleEndian
      #     buff.write_bytes @rgb_data[i].green.to_u16, IO::ByteFormat::LittleEndian
      #     buff.write_bytes @rgb_data[i].blue.to_u16, IO::ByteFormat::LittleEndian
      #     i += 1
      #   end
      #
      # when 3
      #   i = 0
      #   @point_data.each do |p|
      #     x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
      #     y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
      #     z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
      #     buff.write_bytes x, IO::ByteFormat::LittleEndian
      #     buff.write_bytes y, IO::ByteFormat::LittleEndian
      #     buff.write_bytes z, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.intensity, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.bit_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.class_field.value, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.scan_angle.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.user_data.to_u8, IO::ByteFormat::LittleEndian
      #     buff.write_bytes p.point_source_id, IO::ByteFormat::LittleEndian
      #     # Output the gps time
      #     buff.write_bytes @gps_time_data[i], IO::ByteFormat::LittleEndian
      #     # Output the rgb data
      #     buff.write_bytes @rgb_data[i].red.to_u16, IO::ByteFormat::LittleEndian
      #     buff.write_bytes @rgb_data[i].green.to_u16, IO::ByteFormat::LittleEndian
      #     buff.write_bytes @rgb_data[i].blue.to_u16, IO::ByteFormat::LittleEndian
      #     i += 1
      #   end
      # end
      #
      # File.open(@file_name, "w") do |f|
      #   f.write buff.to_slice
      #   f.close
      # end

      # #################################
      # Write the header to the buffer #
      # #################################
      # Output the file signature, LASF
      ptr = @header.file_signature.to_unsafe as UInt8*
      d = ptr.to_slice(4 * sizeof(UInt8))
      d.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the file source ID
      # fsid = @header.file_source_id
      bytes_ptr = pointerof(@header.@file_source_id) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the global encoding
      # ge = @header.global_encoding
      bytes_ptr = pointerof(@header.@global_encoding) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the Project ID 1
      # pid1 = @header.project_id1
      bytes_ptr = pointerof(@header.@project_id1) as {UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # bytes_ptr.value.each do |b, i|
      #   puts("[#{b}, #{i}]")
      # end

      # Output the Project ID 2
      # pid2 = @header.project_id2
      bytes_ptr = pointerof(@header.@project_id2) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the Project ID 3
      # pid3 = @header.project_id3
      bytes_ptr = pointerof(@header.@project_id3) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the Project ID 4
      ptr = @header.project_id4.to_unsafe as UInt8*
      d = ptr.to_slice(8 * sizeof(UInt8))
      d.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the Version major and minor
      buffer[offset] = 1_u8
      offset += 1
      buffer[offset] = 3_u8
      offset += 1

      # Output the System ID
      ptr = fixed_width_string(@header.system_identifier, 32).to_unsafe as UInt8*
      d = ptr.to_slice(32 * sizeof(UInt8))
      d.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the generating software
      ptr = fixed_width_string(@header.generating_software, 32).to_unsafe as UInt8*
      d = ptr.to_slice(32 * sizeof(UInt8))
      d.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the day of the year
      time = Time.now
      @header.file_creation_day = time.day_of_year.to_u16
      # fcd = @header.file_creation_day
      bytes_ptr = pointerof(@header.@file_creation_day) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the year
      @header.file_creation_year = time.year.to_u16
      # fcy = @header.file_creation_year
      bytes_ptr = pointerof(@header.@file_creation_year) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the header size
      hdrs = @header.header_size.to_u16
      # puts typeof(@header.header_size)
      bytes_ptr = pointerof(hdrs) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the offset to points
      otp = @header.offset_to_points.to_u32
      bytes_ptr = pointerof(otp) as {UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the number of VLRs
      nv = @header.number_of_vlrs.to_u32
      bytes_ptr = pointerof(nv) as {UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the point format
      buffer[offset] = @header.point_format
      offset += 1

      # Output the point data record length
      case @header.point_format
      when 0
        @header.point_record_length = 20_u16
      when 1
        @header.point_record_length = 28_u16
      when 2
        @header.point_record_length = 26_u16
      when 3
        @header.point_record_length = 34_u16
      end
      prl = @header.point_record_length.to_u16
      bytes_ptr = pointerof(prl) as {UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the number of points
      np = @header.number_of_points.to_u32
      bytes_ptr = pointerof(np) as {UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the number of points by return
      (0..4).each do |i|
        d32 = @header.number_of_points_by_return[i]
        bytes_ptr = pointerof(d32) as {UInt8, UInt8, UInt8, UInt8}*
        bytes_ptr.value.each do |b|
          buffer[offset] = b
          offset += 1
        end
      end

      # Output the x scale factor
      d64 = @header.x_scale_factor
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the y scale factor
      d64 = @header.y_scale_factor
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the z scale factor
      d64 = @header.z_scale_factor
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the x offset
      d64 = @header.x_offset
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the y offset
      d64 = @header.y_offset
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the z offset
      d64 = @header.z_offset
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the max x
      d64 = @header.max_x
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the min x
      d64 = @header.min_x
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the max y
      d64 = @header.max_y
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the min y
      d64 = @header.min_y
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the max z
      d64 = @header.max_z
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the min z
      d64 = @header.min_z
      bytes_ptr = pointerof(d64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # Output the waveform data start
      d_u64 = @header.waveform_data_start
      bytes_ptr = pointerof(d_u64) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
      bytes_ptr.value.each do |b|
        buffer[offset] = b
        offset += 1
      end

      # ###############################
      # Write the VLRs to the buffer #
      # ###############################

      @vlr_data.each do |vlr|
        d16 = vlr.reserved
        bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
        bytes_ptr.value.each do |b|
          buffer[offset] = b
          offset += 1
        end

        ptr = fixed_width_string(vlr.user_id, 16).to_unsafe as UInt8*
        d = ptr.to_slice(16 * sizeof(UInt8))
        d.each do |b|
          buffer[offset] = b
          offset += 1
        end

        d16 = vlr.record_id
        bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
        bytes_ptr.value.each do |b|
          buffer[offset] = b
          offset += 1
        end

        d16 = vlr.record_length_after_header
        bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
        bytes_ptr.value.each do |b|
          buffer[offset] = b
          offset += 1
        end

        ptr = fixed_width_string(vlr.description, 32).to_unsafe as UInt8*
        d = ptr.to_slice(32 * sizeof(UInt8))
        d.each do |b|
          buffer[offset] = b
          offset += 1
        end

        vlr.binary_data.each do |b|
          buffer[offset] = b
          offset += 1
        end
      end

      # #######################
      # Write the point data #
      # #######################
      case @header.point_format
      when 0
        i = 0
        @point_data.each do |p|
          x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
          bytes_ptr = pointerof(x) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
          bytes_ptr = pointerof(y) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
          bytes_ptr = pointerof(z) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          d16 = p.intensity
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          buffer[offset] = p.bit_field.value
          offset += 1
          buffer[offset] = p.class_field.value
          offset += 1
          buffer[offset] = p.scan_angle.to_u8
          offset += 1
          buffer[offset] = p.user_data.to_u8
          offset += 1

          d16 = p.point_source_id
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          i += 1
        end
      when 1
        i = 0
        @point_data.each do |p|
          x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
          bytes_ptr = pointerof(x) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
          bytes_ptr = pointerof(y) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
          bytes_ptr = pointerof(z) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          d16 = p.intensity
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          buffer[offset] = p.bit_field.value
          offset += 1
          buffer[offset] = p.class_field.value
          offset += 1
          buffer[offset] = p.scan_angle.to_u8
          offset += 1
          buffer[offset] = p.user_data.to_u8
          offset += 1

          d16 = p.point_source_id
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          # Output the gps time
          gt = @gps_time_data[i]
          bytes_ptr = pointerof(gt) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          i += 1
        end
      when 2
        i = 0
        @point_data.each do |p|
          x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
          bytes_ptr = pointerof(x) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
          bytes_ptr = pointerof(y) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
          bytes_ptr = pointerof(z) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          d16 = p.intensity
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          buffer[offset] = p.bit_field.value
          offset += 1
          buffer[offset] = p.class_field.value
          offset += 1
          buffer[offset] = p.scan_angle.to_u8
          offset += 1
          buffer[offset] = p.user_data.to_u8
          offset += 1

          d16 = p.point_source_id
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          # Output the r,g,b data
          r = @rgb_data[i].red.to_u16
          bytes_ptr = pointerof(r) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          g = @rgb_data[i].green.to_u16
          bytes_ptr = pointerof(g) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          bl = @rgb_data[i].blue.to_u16
          bytes_ptr = pointerof(bl) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          i += 1
        end
      when 3
        i = 0
        @point_data.each do |p|
          x = ((p.x - @header.x_offset) / @header.x_scale_factor).to_i32
          bytes_ptr = pointerof(x) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          y = ((p.y - @header.y_offset) / @header.y_scale_factor).to_i32
          bytes_ptr = pointerof(y) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          z = ((p.z - @header.z_offset) / @header.z_scale_factor).to_i32
          bytes_ptr = pointerof(z) as {UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          d16 = p.intensity
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end
          buffer[offset] = p.bit_field.value
          offset += 1
          buffer[offset] = p.class_field.value
          offset += 1
          buffer[offset] = p.scan_angle.to_u8
          offset += 1
          buffer[offset] = p.user_data.to_u8
          offset += 1

          d16 = p.point_source_id
          bytes_ptr = pointerof(d16) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          # Output the gps time
          gt = @gps_time_data[i]
          bytes_ptr = pointerof(gt) as {UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          # Output the r,g,b data
          r = @rgb_data[i].red.to_u16
          bytes_ptr = pointerof(r) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          g = @rgb_data[i].green.to_u16
          bytes_ptr = pointerof(g) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          bl = @rgb_data[i].blue.to_u16
          bytes_ptr = pointerof(bl) as {UInt8, UInt8}*
          bytes_ptr.value.each do |b|
            buffer[offset] = b
            offset += 1
          end

          i += 1
        end
      end

      File.open(@file_name, "w") do |f|
        f.write buffer
        f.close
      end
    end

    # Returns the X,Y,Z data for a specified *point_num*.
    def get_xyz_data(point_num : Int)
      point_num = 0 if point_num < 0
      point_num = @header.number_of_points - 1 if point_num > @header.number_of_points - 1
      return XYZData.new @point_data[point_num].x, @point_data[point_num].y, @point_data[point_num].z
    end

    # Returns the X,Y,Z, and intensity data for a specified *point_num*.
    def get_xyzi_data(point_num : Int)
      point_num = 0 if point_num < 0
      point_num = @header.number_of_points - 1 if point_num > @header.number_of_points - 1
      return XYZIData.new @point_data[point_num].x, @point_data[point_num].y,
        @point_data[point_num].z, @point_data[point_num].intensity
    end

    # Returns the full point record for a specifed *point_num*.
    def [](point_num : Int)
      point_num = 0 if point_num < 0
      point_num = @header.number_of_points - 1 if point_num > @header.number_of_points - 1
      case @header.point_format
      when 1
        return PointRecord1.new @point_data[point_num], @gps_time_data[point_num]
      when 2
        return PointRecord2.new @point_data[point_num], @rgb_data[point_num]
      when 3
        return PointRecord3.new @point_data[point_num], @gps_time_data[point_num], @rgb_data[point_num]
      else # 0
        return PointRecord0.new @point_data[point_num]
      end
    end

    private def fixed_width_string(s : String, width)
      return (s + "\0" * width)[0, width]
    end
  end

  # LAS File Point Record Type 0.
  struct PointRecord0
    def initialize(@point : PointData)
    end

    property point

    def to_s
      return @point.to_s
    end

    def clone
      return PointRecord0.new @point
    end
  end

  # LAS File Point Record Type 1.
  struct PointRecord1
    def initialize(@point : PointData, @gps_time : Float64)
    end

    property point
    property gps_time

    def to_s
      return "#{@point.to_s}, GPS time=" + sprintf("%.6f", @gps_time)
    end

    def clone
      return PointRecord1.new @point, @gps_time
    end
  end

  # LAS File Point Record Type 2.
  struct PointRecord2
    def initialize(@point : PointData, @rgb_data : RGBData)
    end

    property point
    property rgb_data

    def to_s
      return "#{@point.to_s}, RGB=#{@rgb_data.to_s}"
    end

    def clone
      return PointRecord2.new @point, @rgb_data
    end
  end

  # LAS File Point Record Type 3.
  struct PointRecord3
    def initialize(@point : PointData, @gps_time : Float64, @rgb_data : RGBData)
    end

    property point
    property gps_time
    property rgb_data

    def to_s
      return "#{@point.to_s}, GPS time=" + sprintf("%.6f", @gps_time) + ", RGB=#{@rgb_data.to_s}"
    end

    def clone
      return PointRecord2.new @point, @gps_time, @rgb_data
    end
  end
end
