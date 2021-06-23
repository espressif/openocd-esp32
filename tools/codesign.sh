#!/bin/sh

# exit when any command fails
set -e

KEYCHAIN_NAME="openocd.keychain"

echo $MACOS_CERTIFICATE | base64 --decode > $PWD/certificate.p12

security create-keychain -p $KEYCHAIN_PWD $KEYCHAIN_NAME || true

security unlock-keychain -p $KEYCHAIN_PWD $KEYCHAIN_NAME

security import $PWD/certificate.p12 -k $KEYCHAIN_NAME -P $MACOS_CERTIFICATE_PWD -T /usr/bin/codesign

security set-key-partition-list -S apple-tool:,apple:,codesign: -s -k $KEYCHAIN_PWD $KEYCHAIN_NAME

keychains=$(security list-keychains -d user) 
for keychain in $keychains; do 
    kec=$(basename $keychain)
    IFS='-' read -r -a user_kec <<< "$kec"
    if [ ${user_kec} != $KEYCHAIN_NAME ]; then
        list+=${user_kec}
        list+=' '
    fi
done

list+=$KEYCHAIN_NAME

security list-keychains -d user -s ${list[@]}

security find-identity -v

/usr/bin/codesign --force --options runtime -s $IDENTITY_ID $TARGET_BINARY -v 

security delete-keychain $KEYCHAIN_NAME

codesign -dvv ${TARGET_BINARY}

rm $PWD/certificate.p12
