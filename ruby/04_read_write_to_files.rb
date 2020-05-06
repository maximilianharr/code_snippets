# Writing into a file
write_handler = File.new("res/HandlerOutput.txt", "w")

write_handler.puts("Hello world!").to_s

write_handler.close

data_from_file = File.read("res/HandlerOutput.txt")

puts "Data From File: " + data_from_file
