#!/bin/bash

# Generates esp108_regs.h from the xtensa overlay tar contents.
# Instructions:
# Extract gdb/gdb/xtensa-config.c and gdb/regformats/reg-xtensa.dat and put
# them in the same directory as this script. Run the script and the generated
# esp108_regs.h is sent to stdout.

#Warning: this does not modify the FreeRTOS bit, that needs separate modification.


function parseRegs() {
	#Let cpp parse the xtensa_config.c register definitions, then parse the data it spit out.
	{
		echo '#define XTREG(ix, ofs, bi, sz, al, tgbo, flags, cp, typ, group, name, a,b,c,d,e,f) REGDEF name ix typ tgbo group'
		cat xtensa-config.c | grep -v '#include'
	} | cpp | grep REGDEF |  while read dummy name ix flag tgbo group; do 
		if [ -n "$name" ] && ! [ "$name" = "#" ]; then
			tp=XT_REG_GENERAL
			flags="0"
			cmt=""
			nmcap=`echo $name | tr a-z A-Z`
			rtp=$((tgbo>>8))
			hwreg=`printf "0x%02X" $((tgbo&255))`
			if [ $rtp -eq 1 ]; then tp=XT_REG_GENERAL; fi
			if [ $rtp -eq 2 ]; then tp=XT_REG_SPECIAL; fi
			if [ $rtp -eq 3 ]; then tp=XT_REG_USER; fi
			if [ $rtp -eq 32 ]; then tp=XT_REG_ALIASED; fi
			if [ $rtp -eq 0 ]; then 
					tp=XT_REG_RELGEN; 
					hwreg="XT_REG_IDX_AR"$((tgbo&255))
			fi
			if [ $((group)) -eq $((0x1010)) ]; then tp=IGNORE; fi
			if [ $((group)) -eq $((0x401)) ]; then 
					tp=XT_REG_FR
					flags=XT_REGF_COPROC0
					hwreg=`printf "0x%02X" $(((tgbo&255)-48))`
			fi
			if [ "$name" = "fcr" ]; then flags=XT_REGF_COPROC0; fi
			if [ "$name" = "fsr" ]; then flags=XT_REGF_COPROC0; fi
			if [ "$name" = "intset" ]; then flags="XT_REGF_NOREAD"; fi
			if [ "$name" = "intclear" ]; then flags="XT_REGF_NOREAD"; fi
			if [ "$name" = "ddr" ]; then 
				flags="XT_REGF_NOREAD"
				tp=XT_REG_DEBUG
			fi
			if [ "$name" = "mmid" ]; then flags="XT_REGF_NOREAD"; fi
			if [ "$name" = "pc" ]; then
				tp=XT_REG_SPECIAL
				hwreg="176+XT_DEBUGLEVEL"
				cmt="//actually epc[debuglevel]"
			fi
			if [ "$name" = "ps" ]; then
				hwreg="0xC6"
				cmt="//actually EPS[debuglevel]"
			fi
			if [ "$name" = "a0" ]; then cmt=" //WARNING: For these registers, regnum points to the"; fi
			if [ "$name" = "a1" ]; then cmt=" //index of the corresponding ARx registers, NOT to"; fi
			if [ "$name" = "a2" ]; then cmt=" //the processor register number!"; fi
			if ! [ "$tp" = "IGNORE" ]; then
				if [ "$1" = "idx" ]; then
					if [ "$nmcap" = "PC" ]; then
						nmcap="PC=0"
					fi
					printf "\tXT_REG_IDX_%s,\n" $nmcap
				else
					printf "\t{ \"%s\",\t\t\t\t%s, %s, %s }, %s\n" $name $hwreg $tp $flags "$cmt"
				fi
			fi
		fi
	done
}

no=`parseRegs | wc -l`
nogdb=`cat reg-xtensa.dat | grep '32:' | wc -l`

cat <<EOF
//Generated with esp108-genreg.sh

//Xtensa register list taken from gdb/gdb/xtensa-config.c
//gdb wants the registers in the order gdb/regformats/reg-xtensa.dat describes
//them. The enum and esp108_regs structs should be in the same order.

#define XT_NUM_REGS ($no)

//Number of registers returned directly by the G command
//Corresponds to the amount of regs listed in regformats/reg-xtensa.dat in the gdb source
#define XT_NUM_REGS_G_COMMAND ($nogdb)

enum xtensa_reg_idx {
EOF
parseRegs idx
cat <<EOF
};


static const struct esp108_reg_desc esp108_regs[XT_NUM_REGS] = {
EOF
parseRegs
cat <<EOF
};

EOF

