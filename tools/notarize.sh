#!/bin/sh

export PRIMARY_BUNDLE_ID="com.espressif.openocd.zip"

#Target binary path to notarize
/usr/bin/ditto -c -k ${TARGET_BINARY} ${TARGET_BINARY}.zip

#Submit the zipball and get REQUEST_UUID
export SUBMISSION_INFO=$(xcrun altool --notarize-app --primary-bundle-id=${PRIMARY_BUNDLE_ID} -u ${APPLE_ID} -p ${APP_SPECIFIC_PASSWORD} --file ${TARGET_BINARY}.zip 2>&1)
if [ $? != 0 ]; then
    rm $TARGET_BINARY.zip
    echo "Submission failed: $SUBMISSION_INFO \n"
    exit 5
fi

rm $TARGET_BINARY.zip

REQUEST_UUID=$(echo ${SUBMISSION_INFO} | awk -F ' = ' '/RequestUUID/ {print $2}')
if [ -z "${REQUEST_UUID}" ]; then
    echo "Errors trying to upload ${TARGET_BINARY}.zip: ${SUBMISSION_INFO}"
    exit 6
fi
echo "REQUEST_UUID=$REQUEST_UUID";

TRY_CNT=60  # 10 min timeout
for i in $(seq 1 $TRY_CNT)
do
    res_str=$(xcrun altool --notarization-info ${REQUEST_UUID} --username ${APPLE_ID} --password ${APP_SPECIFIC_PASSWORD} --output-format xml)
    echo $res_str | grep -q 'Package Approved';
    if [ $? -eq 0 ]; then
        echo "Package Approved"
        break
    else
        echo "Waiting for package approval...: $i"
        sleep 10;
    fi
done

if [ $i -eq $TRY_CNT ]; then
    echo "Notarization info timed out! Printing latest response"
    echo $res_str
    exit 7
fi

# 5 min timeout
for i in $(seq 1 $TRY_CNT)
do
    res_str=$(xcrun altool --notarization-info ${REQUEST_UUID} --username ${APPLE_ID} --password ${APP_SPECIFIC_PASSWORD} --output-format xml)
    echo $res_str | grep -q 'https://osxapps-ssl.itunes.apple.com/itunes-assets';
    if [ $? -eq 0 ]; then
        echo "Notarization status url is loaded"
        break
    else
        echo "Waiting for notarization status URL: $i"
        sleep 5;
    fi
done

if [ $i -eq $TRY_CNT ]; then
    echo "Notarization status URL timed out!"
    exit 8
fi

echo "Get logfileurl and make sure it doesn't have any issues"
logfileurl=$(xcrun altool --notarization-info $REQUEST_UUID --username ${APPLE_ID} --password ${APP_SPECIFIC_PASSWORD} --output-format xml | xq .plist.dict.dict.string[1] | xargs)
echo "Notarization LogFileURL=$logfileurl for REQUEST_UUID=$REQUEST_UUID ";
log=$(curl -sSL $logfileurl)
issues=$(echo ${log} | jq -r .issues )
if [ "$issues" != "null" ]; then
    printf "There are issues with the notarization (${issues}), see $logfileurl\n"
    printf "=== Log output === \n${log}\n"
    exit 9;
fi;
