#####################################################################
# @copyright (c) 2019 Daimler AG and Robert Bosch GmbH <br>
# The reproduction, distribution and utilization of this file as
# well as the communication of its contents to others without express
# authorization is prohibited. Offenders will be held liable for the
# payment of damages and can be prosecuted. All rights reserved
# particularly in the event of the grant of a patent, utility model
# or design.
#
#####################################################################
# Documentation
# https://api.deepmap.com/api/docs/#/
set -e 

## USER-PARAMETERS ################################################
DEEPMAP_MAP_LOCATION=/lhome/${USER}/artifacts/mapdb/deepmap
DEEPMAP_PW=empty
DEEPMAP_USER=$(git config --global user.email)
DEEPMAP_TOKEN=empty

## PARAMETERS #####################################################
DEEPMAP_FORMAT=athena
DEEPMAP_TOKEN_DURATION=12h00m

DEEPMAP_ID_Boeblingen_Vaihingen=100
DEEPMAP_NAME_Boeblingen_Vaihingen=deepmap-bb_vh

DEEPMAP_ID_Castle_Map=128
DEEPMAP_NAME_Castle_Map=deepmap-castle

DEEPMAP_ID_Daimler_San_Jose=67
DEEPMAP_NAME_Daimler_San_Jose=deepmap-sj

DEEPMAP_ID_Immendingen=98
DEEPMAP_NAME_Immendingen=deepmap-im

DEEPMAP_ID_Renningen=99
DEEPMAP_NAME_Renningen=deepmap-rn

DEEPMAP_ID_Sunnyvale_Daimler=2
DEEPMAP_NAME_Sunnyvale_Daimler=deepmap-sv

DEEPMAP_ID_Synthetic_Map=193
DEEPMAP_NAME_Synthetic_Map=deepmap-synthetic

DEEPMAP_ID_Immendingen_US_Traffic_Lights=191
DEEPMAP_NAME_Immendingen_US_Traffic_Lights=deepmap-im_us

# BOOLEANS | 1=True
UPDATE_TOKEN=1
DOWNLOAD_MAPS=0
LOAD_MAPS_SQL=1

### LOAD SCRIPTS FROM ROS ENVIRONMENT #############################
source $(rospack find map_diff)/include/functions.sh
source $(rospack find map_diff)/include/variables.sh

###################################################################
# Get Token [​/api​/auth​/v1​/login]
if [ ${UPDATE_TOKEN} -eq 1 ]; then
  printf "${RDUAM_GREEN}Updating Deepmap Tocken ${RDUAM_NC}\n"q
  echo "User-name:"
  read DEEPMAP_USER
  echo "DeepMap User-password:"
  read -s DEEPMAP_PW

  DEEPMAP_TOKEN=$(\
    curl -X POST "https://api.deepmap.com/api/auth/v1/login" \
    -H  "accept: application/json" -H  "Content-Type: application/json" \
    -d "{\"duration\":\"${DEEPMAP_TOKEN_DURATION}\",\"password\":\"${DEEPMAP_PW}\",\"username\":\"${DEEPMAP_USER}\"}"  \
    | python3 -c "import sys, json; print(json.load(sys.stdin)['token'])")
  printf "${RDUAM_GREEN}...Finished.${RDUAM_NC}\n"
else
  # Get user input
  printf "${RDUAM_YELLOW}Warning: Using provided token. Please ensure that token is not outdated. ${RDUAM_NC}\n"
fi

echo ${DEEPMAP_TOKEN}

###################################################################
# Get map versions [/api/maps/v1/maps]

printf "${RDUAM_GREEN}Getting map versions: ${RDUAM_NC}\n"

DEEPMAP_VERSION_Boeblingen_Vaihingen=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Boeblingen_Vaihingen}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Castle_Map=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Castle_Map}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Daimler_San_Jose=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Daimler_San_Jose}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Immendingen=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Immendingen}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Renningen=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Renningen}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Sunnyvale_Daimler=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Sunnyvale_Daimler}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Synthetic_Map=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_ID_Synthetic_Map}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')
DEEPMAP_VERSION_Immendingen_US_Traffic_Lights=$(curl -X GET "https://api.deepmap.com/api/maps/v1/${DEEPMAP_NAME_Synthetic_Map}/versions" -H  "accept: application/json" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}" | python -c 'import sys,json;data=json.loads(sys.stdin.read()); print data[0]["version"]')

printf "${RDUAM_GREEN}...Finished.${RDUAM_NC}\n"

###################################################################
# Get Maps [/api/maps/v1/{map_id}/distribution]
mkdir -p ${DEEPMAP_MAP_LOCATION}

function downloadDB(){
  # Read arguments
  DB_NAME=empty
  DB_ID=empty
  DB_VERSION=empty

  POSITIONAL=()
    while [[ $# -gt 0 ]]
    do
        key="$1"
        case "$key" in
          --dbname)
              DB_NAME="$2"
              shift
              shift
              ;;
          --dbid)
              DB_ID="$2"
              shift
              shift
              ;;
          --dbversion)
              DB_VERSION="$2"
              shift
              shift
              ;;
          *)
              echo "Invalid option $key"
              POSITIONAL+=("$1")
              shift
              exit 0
          ;;
      esac
  done

  if ls /${DEEPMAP_MAP_LOCATION}/${DB_NAME}-v${DB_VERSION}.tar.gz 1> /dev/null 2>&1; then
    printf "${RDUAM_GREEN}Map already present, no update required: ${DB_NAME} (Version: ${DB_VERSION} | ID: ${DB_ID}) ${RDUAM_NC}\n"
  else
    printf "${RDUAM_YELLOW}Updating map: ${DB_NAME} (Version: ${DB_VERSION} | ID: ${DB_ID}) ${RDUAM_NC}\n"
    curl --output ${DEEPMAP_MAP_LOCATION}/${DB_NAME}-v${DB_VERSION}.tar.gz -X GET "https://api.deepmap.com/api/maps/v1/${DB_ID}/distribution?format=${DEEPMAP_FORMAT}"\
      -H  "accept: application/octet-stream" -H  "Authorization: Bearer ${DEEPMAP_TOKEN}"
    cp ${DEEPMAP_MAP_LOCATION}/${DB_NAME}-v${DB_VERSION}.tar.gz ${DEEPMAP_MAP_LOCATION}/${DB_NAME}.tar.gz
  fi
}

if [ ${DOWNLOAD_MAPS} -eq 1 ]; then

  # DEEPMAP_ID_Boeblingen_Vaihingen
  downloadDB --dbname ${DEEPMAP_NAME_Boeblingen_Vaihingen} --dbversion ${DEEPMAP_VERSION_Boeblingen_Vaihingen} --dbid ${DEEPMAP_ID_Boeblingen_Vaihingen}
  downloadDB --dbname ${DEEPMAP_NAME_Castle_Map}           --dbversion ${DEEPMAP_VERSION_Castle_Map}           --dbid ${DEEPMAP_ID_Castle_Map}
  downloadDB --dbname ${DEEPMAP_NAME_Daimler_San_Jose}     --dbversion ${DEEPMAP_VERSION_Daimler_San_Jose}     --dbid ${DEEPMAP_ID_Daimler_San_Jose}
  downloadDB --dbname ${DEEPMAP_NAME_Immendingen}          --dbversion ${DEEPMAP_VERSION_Immendingen}          --dbid ${DEEPMAP_ID_Immendingen}
  downloadDB --dbname ${DEEPMAP_NAME_Renningen}            --dbversion ${DEEPMAP_VERSION_Renningen}            --dbid ${DEEPMAP_ID_Renningen}
  downloadDB --dbname ${DEEPMAP_NAME_Sunnyvale_Daimler}    --dbversion ${DEEPMAP_VERSION_Sunnyvale_Daimler}    --dbid ${DEEPMAP_ID_Sunnyvale_Daimler}
  downloadDB --dbname ${DEEPMAP_NAME_Synthetic_Map}        --dbversion ${DEEPMAP_VERSION_Synthetic_Map}        --dbid ${DEEPMAP_ID_Synthetic_Map}
  downloadDB --dbname ${DEEPMAP_NAME_Immendingen_US_Traffic_Lights} --dbversion ${DEEPMAP_VERSION_Immendingen_US_Traffic_Lights} --dbid ${DEEPMAP_ID_Immendingen_US_Traffic_Lights}

fi

###################################################################
# Load maps as Postgres DB
DEEPMAP_TAR_FOLDERNAME=compile_result
DEEPMAP_TAR_FILENAME=pg_dump.txt

function restoreDeepmapDB(){
  # Read arguments
  DB_NAME=empty
  DB_FILE=empty

  POSITIONAL=()
    while [[ $# -gt 0 ]]
    do
        key="$1"
        case "$key" in
          --dbname)
              DB_NAME="$2"
              shift
              shift
              ;;
          --dbfile)
              DB_FILE="$2"
              shift
              shift
              ;;
          *)
              echo "Invalid option $key"
              POSITIONAL+=("$1")
              shift
              exit 0
          ;;
      esac
  done

  printf "${RDUAM_GREEN}Restoring Map ${DB_NAME} ${RDUAM_NC}\n"

  tar -xvf  ${DEEPMAP_MAP_LOCATION}/${DB_NAME}.tar.gz -C ${DEEPMAP_MAP_LOCATION}
  PGPASSWORD=postgres psql -U postgres -h localhost -p 5432 --command="SELECT pg_terminate_backend(pid) FROM pg_stat_activity WHERE pid <> pg_backend_pid() AND datname = '${DB_NAME}'";
  PGPASSWORD=postgres dropdb -h localhost -p 5432 -U postgres ${DB_NAME}
  PGPASSWORD=postgres createdb -h localhost -p 5432 -U postgres ${DB_NAME}

  # try both pg_restore and psql
  #PGPASSWORD=postgres pg_restore -h localhost -p 5432 -U postgres -d ${DB_NAME} -v ${DB_FILE}
  PGPASSWORD=postgres psql -h localhost -p 5432 -U postgres -d ${DB_NAME} -f ${DB_FILE}

  rm -r $(dirname ${DB_FILE})
}

# Create a DB with and without version suffix (e.g. deepmap-im and deepmap-im-v1.0.8)
# @todo The dump of Deepmap frequently changes (pg_dump.txt, pg_dump.sql, Marvin_db_dump_result, etc.) > Write a loop that checks the map in restoreDeepmapDB!
if [ ${LOAD_MAPS_SQL} -eq 1 ]; then

  # ... with version xuffix
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Boeblingen_Vaihingen}-v${DEEPMAP_VERSION_Boeblingen_Vaihingen} --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Castle_Map}-v${DEEPMAP_VERSION_Castle_Map}                     --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Daimler_San_Jose}-v${DEEPMAP_VERSION_Daimler_San_Jose}         --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Immendingen}-v${DEEPMAP_VERSION_Immendingen}                   --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Renningen}-v${DEEPMAP_VERSION_Renningen}                       --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Sunnyvale_Daimler}-v${DEEPMAP_VERSION_Sunnyvale_Daimler}       --dbfile ${DEEPMAP_MAP_LOCATION}/Marvin_compile_result/Marvin_db_dump_result
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Synthetic_Map}-v${DEEPMAP_VERSION_Synthetic_Map}               --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  #restoreDeepmapDB --dbname ${DEEPMAP_NAME_Immendingen_US_Traffic_Lights}-v${DEEPMAP_VERSION_Immendingen_US_Traffic_Lights} --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}

  # ... without version suffix
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Boeblingen_Vaihingen} --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Castle_Map}           --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Daimler_San_Jose}     --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Immendingen}          --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Renningen}            --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Sunnyvale_Daimler}    --dbfile ${DEEPMAP_MAP_LOCATION}/Marvin_compile_result/Marvin_db_dump_result
  restoreDeepmapDB --dbname ${DEEPMAP_NAME_Synthetic_Map}-v${DEEPMAP_VERSION_Synthetic_Map} --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/pg_dump.sql
  #restoreDeepmapDB --dbname ${DEEPMAP_NAME_Immendingen_US_Traffic_Lights}-v${DEEPMAP_VERSION_Immendingen_US_Traffic_Lights} --dbfile ${DEEPMAP_MAP_LOCATION}/${DEEPMAP_TAR_FOLDERNAME}/${DEEPMAP_TAR_FILENAME}

fi
