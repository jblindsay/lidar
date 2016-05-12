require "./spec_helper"
require "../src/lidar/*"

describe Lidar do
  it "works" do
    file_name = "/Users/johnlindsay/Documents/Programming/CrystalCode/lidar/spec/test_data/test.las"
    lf = Lidar::LasFile.new file_name, "r"
    puts("#{lf}")

    # Now print the VLRs
    if lf.header.number_of_vlrs > 0
      (0...lf.header.number_of_vlrs).each { |i|
        puts "VLR #{i + 1}:"
        puts("#{lf.vlr_data[i]}")
      }
    end

    (0...10).each do |i|
      puts("Point #{i + 1} #{lf.get_xyzi_data(i)}")
    end

    # Create a new LAS file
    file_name2 = "/Users/johnlindsay/Documents/Programming/CrystalCode/lidar/spec/test_data/test2.las"
    lf2 = Lidar::LasFile.new file_name2, "w"

    # Add the header
    lf2.add_header(lf.header)

    # Add the VLRs to the file
    lf.vlr_data.each do |vlr|
      lf2.add_vlr(vlr)
    end

    # Add the point data to the file
    north = 4363700.0
    south = 4363200.0
    east = 659400.0
    west = 658900.0
    num_points_added = 0
    (0...lf.header.number_of_points).each do |i|
      p = lf[i]
      if (west <= p.point.x <= east) && (south <= p.point.y <= north)
        lf2.add_point_record(lf[i])
        num_points_added += 1
      end
    end
    puts("num points addded = #{num_points_added}")

    lf2.write

    # now delete the created file to clean up.
    if File.exists?(file_name2)
      File.delete(file_name2)
    end
  end
end
