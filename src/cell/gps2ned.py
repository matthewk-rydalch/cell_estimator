# # reference longitude and latitude
# lon_ref = -111.634873 * val.d2r    # lon <-- rLam
# lat_ref = 40.267766 * val.d2r     # lat <-- rPhi
# alt_ref = 1500                        # range --> height? ... or radar height?


lon_ref = -111.6489643 * val.d2r
lat_ref = 40.2465549 * val.d2r
alt_Ref = 1404.282


# Convert the angles into Earth Centered Earth Fixed Reference Frame

# TO DO : check if x and y are correct
chi = np.sqrt( 1 - val.e_frst_num_ecc * np.sin(lat_ref) * np.sin(lat_ref) )
xr = ( val.e_smaj / chi + r_h ) * np.cos(lat_ref) * np.cos(lon_ref)
yr = ( val.e_smaj / chi + r_h ) * np.cos(lat_ref) * np.sin(lon_ref)
zr = ( val.e_smaj * (1 - val.e_frst_num_ecc) / chi + r_h ) * np.sin(lat_ref)

def GPS2NED(lon, lat, alt, lon_ref, lat_ref, alt_ref, xr, yr, zr):
    lon_deg = lon   # Longitude
    lat_deg = lat     # Latitude
    alt = alt         # Altitude

    #send in lon (lambda, or longitude) as an angle in degrees North ex. 76.42816
    #send in lat (phi, or latitude) as an angle in degrees East ex.38.14626
    #both of those are converted into radians
    #send in alt as a altitude (mean sea level) provo = about 1500

    # lon_ref <-- rLam is the reference longitude (in RADIANS)
    # lat_ref <-- rPhi is the reference latitude (in RADIANS)
    # The reference angles define where the 0,0,0 point is in the local NED coordinates


    chi = np.sqrt(1 - val.e_frst_num_ecc * np.sin(lat_ref)**2 )

    #Convert the incoming angles to radians
    lon = lon_deg * val.d2r         # typical longitude greek is lambda
    lat = lat_deg * val.d2r         # " " latitude greek is phi

    # Convert the angles into Earth Centered Earth Fixed Reference Frame
    x = ( val.e_smaj / chi + alt)* np.cos(lat)*np.cos(lon)
    y = ( val.e_smaj / chi + alt)* np.cos(lat)*np.sin(lon)
    z = ( val.e_smaj * (1 - val.e_frst_num_ecc) / chi + alt) * np.sin(lat)

    # Find the difference between the point x, y, z to the reference point in ECEF
    dx = x - xr
    dy = y - yr
    dz = z - zr

    # Rotate the point in ECEF to the Local NED
    N = (-np.sin(lat_ref)*np.cos(lon_ref)*dx) + (-np.sin(lat_ref)*np.sin(lon_ref)*dy) + np.cos(lat_ref)*dz
    E = (np.sin(lon_ref)*dx) - (np.cos(lon_ref)*dy)
    D = (-np.cos(lat_ref)*np.cos(lon_ref)*dx) + (-np.cos(lat_ref)*np.sin(lon_ref)*dy) + (-np.sin(lat_ref)*dz)

    ned = np.array([[N,E,D]])
    return ned

# lon = -111.634873
lat = 40.246588
lon = -111.648979
# lat = 40.267766 

GPS2NED(lon,lat,alt_ref,lon_ref,lat_ref,alt_ref, xr, yr, zr)
