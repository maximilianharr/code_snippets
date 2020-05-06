# Operators
# Comparison : == != < > <= >=
# Logical : && || ! and or not

value = 12

if (value > 10) && (value < 15)
	puts "Value between 10 and 15"
elsif (value >= 15)
	puts "Value above 15"
else
	puts "Value bewow 10"
end

# unless
unless value < 15
	puts "Value above 15"
else
	puts "Value below 15"
end

# case
case value
when 12
	puts "case value is 12"
else
	puts "case value is not 12"
end

# Ternary operator
puts ( value >= 50 ) ? "Bigger than fifty" : "Smaller than fifty"

