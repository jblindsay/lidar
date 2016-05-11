# lidar

This is a small library for reading and writing [LiDAR](https://en.wikipedia.org/wiki/Lidar) data in the widely used [LAS file format](http://www.asprs.org/committee-general/laser-las-file-format-exchange-activities.html). The library has been developed as an experiment to learn the [Crystal programming language](http://crystal-lang.org/).

## Installation


Add this to your application's `shard.yml`:

```yaml
dependencies:
  lidar:
    github: jblindsay/lidar
```


## Usage


```crystal
require "lidar"

file_name = "/Users/johnlindsay/Documents/Programming/CrystalCode/lidar/spec/test_data/test.las"
lf = Lidar::LasFile.new file_name, "r"
puts("#{lf.to_s}")

# Now print the VLRs
if lf.header.number_of_vlrs > 0
  (0...lf.header.number_of_vlrs).each { |i|
    puts "VLR #{i + 1}:"
    puts("#{lf.vlr_data[i].to_s}")
  }
end

(0...10).each do |i|
  puts("Point #{i + 1} #{lf.get_xyzi_data(i).to_s}")
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
```


<!-- TODO: Write usage instructions here -->

<!-- ## Development

TODO: Write development instructions here -->

## Contributing

1. Fork it ( https://github.com/jblindsay/lidar/fork )
2. Create your feature branch (git checkout -b my-new-feature)
3. Commit your changes (git commit -am 'Add some feature')
4. Push to the branch (git push origin my-new-feature)
5. Create a new Pull Request

## Contributors

- [jblindsay](https://github.com/jblindsay) John Lindsay - creator, maintainer
