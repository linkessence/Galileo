/*++
 
Copyright (c) 2013 Intel Corporation.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
* Neither the name of Intel Corporation nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Module Name:

  MiscBiosVendorFunction.c
  
Abstract: 

  BIOS vendor information boot time changes.
  Misc. subclass type 2.
  SMBIOS type 0.

--*/


#include "CommonHeader.h"

#include "SmbiosMisc.h"

/**
  This function returns the value & exponent to Base2 for a given
  Hex value. This is used to calculate the BiosPhysicalDeviceSize.

  @param Value                      The hex value which is to be converted into value-exponent form
  @param Exponent                   The exponent out of the conversion

  @retval EFI_SUCCESS               All parameters were valid and *Value & *Exponent have been set.
  @retval EFI_INVALID_PARAMETER     Invalid parameter was found.
  
**/
EFI_STATUS  
GetValueExponentBase2(
  IN OUT UINTN        *Value,
  OUT    UINTN        *Exponent
  )
{
  if ((Value == NULL) || (Exponent == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  
  while ((*Value % 2) == 0) {
    *Value=*Value/2;
    (*Exponent)++;
  }

  return EFI_SUCCESS;
}

/**
  Field Filling Function. Transform an EFI_EXP_BASE2_DATA to a byte, with '64k'
  as the unit.

  @param  Base2Data              Pointer to Base2_Data

  @retval EFI_SUCCESS            Transform successfully.       
  @retval EFI_INVALID_PARAMETER  Invalid parameter was found.    

**/
UINT16
Base2ToByteWith64KUnit (
  IN      EFI_EXP_BASE2_DATA  *Base2Data
  )
{  
  UINT16              Value;
  UINT16              Exponent;

  Value     = Base2Data->Value;
  Exponent  = Base2Data->Exponent;
  Exponent -= 16;
  Value <<= Exponent;

  return Value;
}


/**
  This function makes boot time changes to the contents of the
  MiscBiosVendor (Type 0).

  @param  RecordData                 Pointer to copy of RecordData from the Data Table.

  @retval EFI_SUCCESS                All parameters were valid.
  @retval EFI_UNSUPPORTED            Unexpected RecordType value.
  @retval EFI_INVALID_PARAMETER      Invalid parameter was found.

**/
MISC_SMBIOS_TABLE_FUNCTION(MiscBiosVendor)
{
  CHAR8                 *OptionalStrStart;
  UINTN                 VendorStrLen;
  UINTN                 VerStrLen;
  UINTN                 DateStrLen;
  UINTN                 BiosPhysicalSizeHexValue;
  UINTN                 BiosPhysicalSizeExponent;
  CHAR16                Version[SMBIOS_STRING_MAX_LENGTH];
  CHAR16                Vendor[SMBIOS_STRING_MAX_LENGTH];
  CHAR16                ReleaseDate[SMBIOS_STRING_MAX_LENGTH];
  EFI_STRING            VersionPtr;
  EFI_STRING            VendorPtr;
  EFI_STRING            ReleaseDatePtr;
  EFI_STATUS            Status;
  STRING_REF            TokenToGet;
  STRING_REF            TokenToUpdate;
  SMBIOS_TABLE_TYPE0    *SmbiosRecord;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  EFI_MISC_BIOS_VENDOR *ForType0InputData;
  EFI_PLATFORM_TYPE_PROTOCOL  *PlatformType;

  BiosPhysicalSizeHexValue = 0x0;
  BiosPhysicalSizeExponent = 0x0;
  ForType0InputData        = (EFI_MISC_BIOS_VENDOR *)RecordData;

  //
  // First check for invalid parameters.
  //
  if (RecordData == NULL) {
    return EFI_INVALID_PARAMETER;
  } 
  //
  // Now update the BiosPhysicalSize
  //
  BiosPhysicalSizeHexValue = PcdGet32 (PcdFlashAreaSize);
  Status= GetValueExponentBase2 (
            &BiosPhysicalSizeHexValue, 
            &BiosPhysicalSizeExponent
            );
  if(Status == EFI_SUCCESS){
    ForType0InputData->BiosPhysicalDeviceSize.Value = (UINT16)BiosPhysicalSizeHexValue;
    ForType0InputData->BiosPhysicalDeviceSize.Exponent = (UINT16)BiosPhysicalSizeExponent;
  }
  //
  // Update strings from PCD
  //
  AsciiStrToUnicodeStr (PcdGetPtr (PcdSMBIOSBiosVendor), Vendor);
  if (StrLen (Vendor) > 0) {     
    TokenToUpdate = STRING_TOKEN (STR_MISC_BIOS_VENDOR);
    HiiSetString (mHiiHandle, TokenToUpdate, Vendor, NULL);
  }
  TokenToGet = STRING_TOKEN (STR_MISC_BIOS_VENDOR);
  VendorPtr = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
  VendorStrLen = StrLen(VendorPtr);
  if (VendorStrLen > SMBIOS_STRING_MAX_LENGTH) {
    return EFI_UNSUPPORTED;
  }

  Status = gBS->LocateProtocol (&gEfiPlatformTypeProtocolGuid, NULL, (VOID **) &PlatformType);
  ASSERT_EFI_ERROR (Status);
  UnicodeSPrint (Version, sizeof (Version), L"0x%08x", PlatformType->FirmwareVersion);
  if (StrLen (Version) > 0) {     
    TokenToUpdate = STRING_TOKEN (STR_MISC_BIOS_VERSION);
    HiiSetString (mHiiHandle, TokenToUpdate, Version, NULL);
  }
  TokenToGet = STRING_TOKEN (STR_MISC_BIOS_VERSION);
  VersionPtr = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
  VerStrLen = StrLen(VersionPtr);
  if (VerStrLen > SMBIOS_STRING_MAX_LENGTH) {
    return EFI_UNSUPPORTED;
  }

  AsciiStrToUnicodeStr (PcdGetPtr (PcdSMBIOSBiosReleaseDate), ReleaseDate);
  if (StrLen (ReleaseDate) > 0) {     
    TokenToUpdate = STRING_TOKEN (STR_MISC_BIOS_RELEASE_DATE);
    HiiSetString (mHiiHandle, TokenToUpdate, ReleaseDate, NULL);
  } 
  TokenToGet = STRING_TOKEN (STR_MISC_BIOS_RELEASE_DATE);
  ReleaseDatePtr = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
  DateStrLen = StrLen(ReleaseDatePtr);
  if (DateStrLen > SMBIOS_STRING_MAX_LENGTH) {
    return EFI_UNSUPPORTED;
  }
  
  //
  // Two zeros following the last string.
  //
  SmbiosRecord = AllocatePool(sizeof (SMBIOS_TABLE_TYPE0) + VendorStrLen + 1 + VerStrLen + 1 + DateStrLen + 1 + 1);
  ZeroMem(SmbiosRecord, sizeof (SMBIOS_TABLE_TYPE0) + VendorStrLen + 1 + VerStrLen + 1 + DateStrLen + 1 + 1);

  SmbiosRecord->Hdr.Type = EFI_SMBIOS_TYPE_BIOS_INFORMATION;
  SmbiosRecord->Hdr.Length = sizeof (SMBIOS_TABLE_TYPE0);
  //
  // Make handle chosen by smbios protocol.add automatically.
  //
  SmbiosRecord->Hdr.Handle = 0;
  //
  // Vendor will be the 1st optional string following the formatted structure.
  //
  SmbiosRecord->Vendor = 1;  
  //
  // Version will be the 2nd optional string following the formatted structure.
  //
  SmbiosRecord->BiosVersion = 2;
  SmbiosRecord->BiosSegment = PcdGet16 (PcdSMBIOSBiosStartAddress);
  //
  // ReleaseDate will be the 3rd optional string following the formatted structure.
  //
  SmbiosRecord->BiosReleaseDate = 3;
  SmbiosRecord->BiosSize = (UINT8)(Base2ToByteWith64KUnit(&ForType0InputData->BiosPhysicalDeviceSize) - 1);
  *(UINT64 *)&SmbiosRecord->BiosCharacteristics = PcdGet64 (PcdSMBIOSBiosChar);
  //
  // CharacterExtensionBytes also store in ForType0InputData->BiosCharacteristics1 later two bytes to save size.
  //
  SmbiosRecord->BIOSCharacteristicsExtensionBytes[0] = PcdGet8 (PcdSMBIOSBiosCharEx1);
  SmbiosRecord->BIOSCharacteristicsExtensionBytes[1] = PcdGet8 (PcdSMBIOSBiosCharEx2);

  SmbiosRecord->SystemBiosMajorRelease = ForType0InputData->BiosMajorRelease;
  SmbiosRecord->SystemBiosMinorRelease = ForType0InputData->BiosMinorRelease;
  SmbiosRecord->EmbeddedControllerFirmwareMajorRelease = ForType0InputData->BiosEmbeddedFirmwareMajorRelease;
  SmbiosRecord->EmbeddedControllerFirmwareMinorRelease = ForType0InputData->BiosEmbeddedFirmwareMinorRelease;

  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr(VendorPtr, OptionalStrStart);
  UnicodeStrToAsciiStr(VersionPtr, OptionalStrStart + VendorStrLen + 1);
  UnicodeStrToAsciiStr(ReleaseDatePtr, OptionalStrStart + VendorStrLen + 1 + VerStrLen + 1);
  //
  // Now we have got the full smbios record, call smbios protocol to add this record.
  //
  SmbiosHandle = SMBIOS_HANDLE_PI_RESERVED;
  Status = Smbios-> Add(
                      Smbios, 
                      NULL,
                      &SmbiosHandle, 
                      (EFI_SMBIOS_TABLE_HEADER *) SmbiosRecord
                      );

  FreePool(SmbiosRecord);
  return Status;
}
