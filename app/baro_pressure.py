## Script for calculating barometric presure differences

# Constants
ps = 101.3 # kPa
z_star = 8404 # m
gamma = 1.4

# Inputs
ground_height = input( "Enter a ground height in meters: " )
height = input( "Enter a height in meters: " )
ground_height = float( ground_height )
height = ground_height + float( height )

# Calculations
gamma_const1 = (gamma - 1.0 )/( gamma)
gamma_const2 = (gamma)/(gamma - 1.0 )
p_g = ps*( ( 1 - gamma_const1*(ground_height/z_star) )**(gamma_const2) )
p   = ps*( ( 1 - gamma_const1*(height/z_star ) )**(gamma_const2) )
dp = p - p_g

# Display Results
print( "Results: " )
print( "Ground Pressure: {:.2f} kPa".format( p_g ) )
print( "Altitude Pressure: {:.2f} kPa".format( p ) )
print( "Pressure Differential: {:.2f} kPa".format(dp) )