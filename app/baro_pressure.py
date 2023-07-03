## Script for calculating barometric presure differences

# Altitude to pressure calculation, alt in ft, pressure in kPa
def alt_to_pressure( alt ):
    # Constants
    ps     = 101.3 # kPa
    z_star = 8404 # m
    gamma  = 1.4

    # Convert to meters
    alt /= 3.28084

    # Calculations
    gamma_const1 = (gamma - 1.0 )/( gamma )
    gamma_const2 = (gamma)/(gamma - 1.0 )
    return ps*( ( 1 - gamma_const1*( alt/z_star ) )**(gamma_const2) )
    
## alt_to_pressure

# Pressure to altitude calculation, pressure in kPa, alt in ft
def pressure_to_alt( pressure ):
    # Constants
    ps     = 101.3  # kPa
    z_star = 8404.0 # m
    gamma  = 1.4

    # Calculations
    gamma_const1 = (gamma - 1.0 )/( gamma )
    gamma_const2 = (gamma)/(gamma - 1.0 )
    alt = z_star*gamma_const2*( 1 - ( ( pressure/ps )**( gamma_const1 ) ) )

    # Convert to feet
    return alt*3.28084
## pressure_to_alt

# Inputs
main_height     = input( "Enter a main deployment altitude in feet: " )
main_height     = float( main_height )
elevation       = input( "Enter an elevation altitude in feet: " )
elevation       = float( elevation )
error_margin    = input( "Enter an error margin in kPa: " )
error_margin    = float( error_margin )
main_height_alt = elevation + main_height

# Calculate pressure at main height 
ground_pressure = alt_to_pressure( elevation )
main_pressure   = alt_to_pressure( main_height_alt )

# Add error margins to ground pressure
ground_alt_high = pressure_to_alt( ground_pressure - error_margin )
ground_alt_low  = pressure_to_alt( ground_pressure + error_margin )

# Calculate altitude differences due to error
main_min_alt = pressure_to_alt( main_pressure + error_margin ) - ground_alt_low 
main_max_alt = pressure_to_alt( main_pressure - error_margin ) - ground_alt_high 
alt_pos_margin = main_max_alt - main_height 
alt_neg_margin = main_height - main_min_alt 

# Display Results
print( "Results: " )
print( "Ground Pressure: {:.2f} kPa".format( ground_pressure ) )
print( "Main Pressure: {:.2f} kPa".format( main_pressure ) )
print( "Max Main Altitude: {:.2f} ft".format( main_max_alt ) )
print( "Min Main Altitude: {:.2f} ft".format( main_min_alt ) )
print( "Upward Margin: {:.2f} ft".format( alt_pos_margin ) )
print( "Downward Margin: {:.2f} ft".format( alt_neg_margin ) )