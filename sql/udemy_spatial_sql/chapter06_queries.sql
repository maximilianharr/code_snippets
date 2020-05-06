-- SELECT * FROM parcels ORDER BY parcel_id ASC LIMIT 5;
-- SELECT asmt - land AS STRUCVALUE, parcelkey FROM parcels;
-- DROP TABLE qlayer;
-- SELECT * INTO qlayer
-- FROM parcels WHERE LEFT(AddrStreet,2) = 'BU'
-- AND RIGHT(AddrStreet,1) = 'E';
SELECT concat('assessed value is:  ', asmt::numeric::money::text) FROM parcels;
