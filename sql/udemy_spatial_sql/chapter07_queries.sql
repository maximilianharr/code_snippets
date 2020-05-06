-- SELECT gps_date FROM trees LIMIT 10;
-- SELECT date_part('day', gps_date) FROM trees;
SELECT to_char(gps_date, 'Day') FROM trees LIMIT 10;
