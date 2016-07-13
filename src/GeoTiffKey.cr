require "io"

# A module for handling GeoTIFF image data and tags
module GeoTiff
  class GeoTiffKey
    @tag_map = {
      254 => GeoTiffTag.new("NewSubFileType", 254),
      256 => GeoTiffTag.new("ImageWidth", 256),
      257 => GeoTiffTag.new("ImageLength", 257),
      258 => GeoTiffTag.new("BitsPerSample", 258),
      259 => GeoTiffTag.new("Compression", 259),
      262 => GeoTiffTag.new("PhotometricInterpretation", 262),
      266 => GeoTiffTag.new("FillOrder", 266),
      269 => GeoTiffTag.new("DocumentName", 269),
      284 => GeoTiffTag.new("PlanarConfiguration", 284),
      270 => GeoTiffTag.new("ImageDescription", 270),
      271 => GeoTiffTag.new("Make", 271),
      272 => GeoTiffTag.new("Model", 272),
      273 => GeoTiffTag.new("StripOffsets", 273),
      274 => GeoTiffTag.new("Orientation", 274),
      277 => GeoTiffTag.new("SamplesPerPixel", 277),
      278 => GeoTiffTag.new("RowsPerStrip", 278),
      279 => GeoTiffTag.new("StripByteCounts", 279),

      282 => GeoTiffTag.new("XResolution", 282),
      283 => GeoTiffTag.new("YResolution", 283),
      296 => GeoTiffTag.new("ResolutionUnit", 296),

      305 => GeoTiffTag.new("Software", 305),
      306 => GeoTiffTag.new("DateTime", 306),

      322 => GeoTiffTag.new("TileWidth", 322),
      323 => GeoTiffTag.new("TileLength", 323),
      324 => GeoTiffTag.new("TileOffsets", 324),
      325 => GeoTiffTag.new("TileByteCounts", 325),

      317 => GeoTiffTag.new("Predictor", 317),
      320 => GeoTiffTag.new("ColorMap", 320),
      338 => GeoTiffTag.new("ExtraSamples", 338),
      339 => GeoTiffTag.new("SampleFormat", 339),

      34735 => GeoTiffTag.new("GeoKeyDirectoryTag", 34735),
      34736 => GeoTiffTag.new("GeoDoubleParamsTag", 34736),
      34737 => GeoTiffTag.new("GeoAsciiParamsTag", 34737),
      33550 => GeoTiffTag.new("ModelPixelScaleTag", 33550),
      33922 => GeoTiffTag.new("ModelTiepointTag", 33922),
      34264 => GeoTiffTag.new("ModelTransformationTag", 34264),
      42112 => GeoTiffTag.new("GDAL_METADATA", 42112),
      42113 => GeoTiffTag.new("GDAL_NODATA", 42113),

       1024 => GeoTiffTag.new("GTModelTypeGeoKey", 1024),
       1025 => GeoTiffTag.new("GTRasterTypeGeoKey", 1025),
       1026 => GeoTiffTag.new("GTCitationGeoKey", 1026),
       2048 => GeoTiffTag.new("GeographicTypeGeoKey", 2048),
       2049 => GeoTiffTag.new("GeogCitationGeoKey", 2049),
       2050 => GeoTiffTag.new("GeogGeodeticDatumGeoKey", 2050),
       2051 => GeoTiffTag.new("GeogPrimeMeridianGeoKey", 2051),
       2061 => GeoTiffTag.new("GeogPrimeMeridianLongGeoKey", 2061),
       2052 => GeoTiffTag.new("GeogLinearUnitsGeoKey", 2052),
       2053 => GeoTiffTag.new("GeogLinearUnitSizeGeoKey", 2053),
       2054 => GeoTiffTag.new("GeogAngularUnitsGeoKey", 2054),
       2055 => GeoTiffTag.new("GeogAngularUnitSizeGeoKey", 2055),
       2056 => GeoTiffTag.new("GeogEllipsoidGeoKey", 2056),
       2057 => GeoTiffTag.new("GeogSemiMajorAxisGeoKey", 2057),
       2058 => GeoTiffTag.new("GeogSemiMinorAxisGeoKey", 2058),
       2059 => GeoTiffTag.new("GeogInvFlatteningGeoKey", 2059),
       2060 => GeoTiffTag.new("GeogAzimuthUnitsGeoKey", 2060),
       3072 => GeoTiffTag.new("ProjectedCSTypeGeoKey", 3072),
       3073 => GeoTiffTag.new("PCSCitationGeoKey", 3073),
       3074 => GeoTiffTag.new("ProjectionGeoKey", 3074),
       3075 => GeoTiffTag.new("ProjCoordTransGeoKey", 3075),
       3076 => GeoTiffTag.new("ProjLinearUnitsGeoKey", 3076),
       3077 => GeoTiffTag.new("ProjLinearUnitSizeGeoKey", 3077),
       3078 => GeoTiffTag.new("ProjStdParallel1GeoKey", 3078),
       3079 => GeoTiffTag.new("ProjStdParallel2GeoKey", 3079),
       3080 => GeoTiffTag.new("ProjNatOriginLongGeoKey", 3080),
       3081 => GeoTiffTag.new("ProjNatOriginLatGeoKey", 3081),
       3082 => GeoTiffTag.new("ProjFalseEastingGeoKey", 3082),
       3083 => GeoTiffTag.new("ProjFalseNorthingGeoKey", 3083),
       3084 => GeoTiffTag.new("ProjFalseOriginLongGeoKey", 3084),
       3085 => GeoTiffTag.new("ProjFalseOriginLatGeoKey", 3085),
       3086 => GeoTiffTag.new("ProjFalseOriginEastingGeoKey", 3086),
       3087 => GeoTiffTag.new("ProjFalseOriginNorthingGeoKey", 3087),
       3088 => GeoTiffTag.new("ProjCenterLongGeoKey", 3088),
       3089 => GeoTiffTag.new("ProjCenterLatGeoKey", 3089),
       3090 => GeoTiffTag.new("ProjCenterEastingGeoKey", 3090),
       3091 => GeoTiffTag.new("ProjFalseOriginNorthingGeoKey", 3091),
       3092 => GeoTiffTag.new("ProjScaleAtNatOriginGeoKey", 3092),
       3093 => GeoTiffTag.new("ProjScaleAtCenterGeoKey", 3093),
       3094 => GeoTiffTag.new("ProjAzimuthAngleGeoKey", 3094),
       3095 => GeoTiffTag.new("ProjStraightVertPoleLongGeoKey", 3095),
       4096 => GeoTiffTag.new("VerticalCSTypeGeoKey", 4096),
       4097 => GeoTiffTag.new("VerticalCitationGeoKey", 4097),
       4098 => GeoTiffTag.new("VerticalDatumGeoKey", 4098),
       4099 => GeoTiffTag.new("VerticalUnitsGeoKey", 4099),
      50844 => GeoTiffTag.new("RPCCoefficientTag", 50844),
      34377 => GeoTiffTag.new("Photoshop", 34377),
    }
  end

  struct GeoTiffTag
    property name, code

    def initialize(@name : String, @code : Int32)
    end

    def to_(io)
      io << "Name: #{@name}, Code: #{@code}"
    end
  end

  # An Image File Directory (IFD) entry.
  struct IfdEntry
    property tag, data_type, count, raw_data

    def initialize(@tag : GeoTiffTag, @data_type : GeotiffDataType, @count : Int32, @raw_data : Array(UInt8))
      # @byteOrder : binary.ByteOrder
    end
  end

  # GeoTIFF data type
  enum GTDataType : UInt8
    GTByte      = 1
    GTASCII
    GTShort
    GTLong
    GTRational
    GTSbyte
    GTUndefined
    GTSshort
    GTSlong
    GTSrational
    GTFloat
    GTDouble
  end

  class GeoTiffDataType
    def initialize(@value : GTDataType)
    end

    @data_type_lengths = [0, 1, 1, 2, 4, 8, 1, 2, 2, 4, 8, 8, 16]

    @data_type_strings = ["Byte", "ASCII", "Short", "Long", "Rational", "Sbyte", "Undefined",
      "Sshort", "Slong", "Srational", "Float", "Double"]

    def get_bit_length
      return @data_type_lengths[@value]
    end
  end
end
