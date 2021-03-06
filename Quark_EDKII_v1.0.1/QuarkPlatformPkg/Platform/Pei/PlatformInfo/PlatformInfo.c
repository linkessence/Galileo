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

  PlatformInfo.c

Abstract:

  Platform Info PEIM.

--*/

#include "PlatformInfo.h"

#define  TEMP_BUS_NUMBER    (0x3F)

static EFI_PEI_PPI_DESCRIPTOR       mPlatformInfoPpi = {
    EFI_PEI_PPI_DESCRIPTOR_PPI | EFI_PEI_PPI_DESCRIPTOR_TERMINATE_LIST,
    &gEfiPlatformInfoGuid,
    NULL
  };

static EFI_GUID mPDatFileNameTable[]  = { PDAT_FILE_NAME_TABLE_DEFINITION };
static UINTN mPDatFileNameTableLen  = ((sizeof(mPDatFileNameTable)) / sizeof (EFI_GUID));

VOID
CharOrIndex (
  IN UINTN                                 GotIndex,
  OUT CHAR8                                *EquivalentChar,
  IN CHAR8                                 GotChar,
  OUT UINTN                                *EquivalentIndex
  )
/*++
Routine Description:

  Translate table index to select key or translate key to table index.

Arguments:

  GotIndex         -  Table index to translate.
  EquivalentChar   -  Key for GotIndex.
  GotChar          -  Key to translate.
  EquivalentIndex  -  Table index for GotChar.

Returns:

  EquivalentChar   -  Key for GotIndex.
  EquivalentIndex  -  Table index for GotChar.

--*/
{
  CHAR8                            Temp;
  if (EquivalentChar != NULL) {
    if (GotIndex < 10) {
      *EquivalentChar = '0' + ((CHAR8) GotIndex);
    } else {
      Temp = ((CHAR8) GotIndex) - 10;
      *EquivalentChar = 'a' + Temp;
    }
    return;
  }
  if (EquivalentIndex != NULL) {
    if (GotChar < 'a') {
      *EquivalentIndex = (UINTN) (GotChar - '0');
    } else {
      Temp = (GotChar - 'a');
      *EquivalentIndex = 10 + (UINTN) Temp;
    }
  }
}

EFI_STATUS
CheckMrcParams (
  IN UINT16     Type,
  IN PDAT_ITEM  *Item
  )
/*++
Routine Description:

  Check mrc config valid.

Arguments:

  Type             -  Type of platform mrc config is for.
  Item             -  Mrc config item to check.

Returns:

  EFI_SUCCESS      -  Valid Mrc config.
  EFI_INCOMPATIBLE_VERSION -  Invalid item version.
  EFI_INVALID_PARAMETER - Mrc config not for platform type specified.

--*/
{
  PDAT_MRC_ITEM                     *MrcItemData;

  if (Item->Header.Version < PDAT_MRC_MIN_VERSION) {
    DEBUG ((EFI_D_ERROR, "Platform Info: Mrc Vars in Platform Data is Version:%d. Must be >= Version:%d!!!!!\n", Item->Header.Version, PDAT_MRC_MIN_VERSION));
    return EFI_INCOMPATIBLE_VERSION;
  }

  MrcItemData = (PDAT_MRC_ITEM *) Item->Data;

  //
  // Check parameters for this platform.
  //
  if (Type != MrcItemData->PlatformId) {
    DEBUG ((EFI_D_ERROR, "Platform Info: Mrc Vars for this platform not found: want id %d got id %d\n", Type, MrcItemData->PlatformId));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
FindAndCheckPlatformDataFile (
  IN CONST EFI_GUID                 *FileNameGuid,
  OUT PDAT_AREA                     **AreaPtr,
  OUT UINTN                         *AreaSize,
  OUT PDAT_ITEM                     **TypeItemPtr,
  OUT PDAT_ITEM                     **MrcItemPtr,
  OUT UINT16                        *TypePtr
  )
/*++
Routine Description:

  Find platform data file in firmware volumes and check it is valid.

Arguments:

  FileNameGuid     -  Platform data file to find.
  AreaPtr          -  Pointer to be pointed to platform data area.
  AreaSize         -  Update with size of platform data area.
  TypeItemPtr      -  Update with address of platform type item.
  MrcItemPtr       -  Update with address of mrc config item.
  TypePtr          -  Store platform type at this address.

Returns:

  EFI_SUCCESS      -  Valid file found.
  EFI_NOT_FOUND    -  File not found.
  EFI_INVALID_PARAMETER - File found but data in file invalid.
  AreaPtr          -  Pointing to platform data area in file.
  AreaSize         -  Updated with size of platform data area.
  TypeItemPtr      -  Updated with address of platform type item.
  MrcItemPtr       -  Updated with address of mrc config item.
  TypePtr          -  Updated with platform type.

--*/
{
  EFI_STATUS              Status;
  Status = PlatformFindFvFileRawDataSection (NULL, FileNameGuid, (VOID **) AreaPtr, AreaSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_INFO, "Platform Info: File %g not found in FV\n", FileNameGuid));
    return EFI_NOT_FOUND;
  }
  Status = PDatLibValidateArea (*AreaPtr, TRUE);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_INFO, "Platform Info: File %g has bad PDAT area %r\n", FileNameGuid, Status));
    return EFI_INVALID_PARAMETER;
  }
  *TypeItemPtr = PDatLibFindItem (*AreaPtr, PDAT_ITEM_ID_PLATFORM_ID, FALSE, TypePtr);
  if (*TypeItemPtr == NULL) {
    DEBUG ((EFI_D_INFO, "Platform Info: File %g has no PID item\n", FileNameGuid));
    return EFI_INVALID_PARAMETER;
  }
  if ((*TypePtr > TypeUnknown)  && (*TypePtr < TypePlatformMax)) {
    *MrcItemPtr = PDatLibFindItem (*AreaPtr, PDAT_ITEM_ID_MRC_VARS, FALSE, NULL);
    if (*MrcItemPtr == NULL) {
      DEBUG ((EFI_D_INFO, "Platform Info: File %g has no MRCCFG item\n", FileNameGuid));
      return EFI_INVALID_PARAMETER;
    }
  } else {
    DEBUG ((EFI_D_INFO, "Platform Info: Invalid platform type %d\n", (UINTN) *TypePtr));
    return EFI_INVALID_PARAMETER;
  }
  return CheckMrcParams (*TypePtr, *MrcItemPtr);
}

VOID
TracePlatformDataSelectList (
  VOID
  )
/*++
Routine Description:

  Trace list of valid platform data files stored in Firmware volumes.

Arguments:

    None

Returns:

    None

--*/
{
  UINTN                   Index;
  EFI_STATUS              Status;
  PDAT_AREA               *CurrArea;
  UINTN                   CurrAreaSize;
  PDAT_ITEM               *CurrTypeItem;
  PDAT_ITEM               *CurrMrcItem;
  UINT16                  CurrType;
  CHAR8                   KeyStrForIndex [4];

  AsciiStrCpy (KeyStrForIndex,"'?'");

  DEBUG ((EFI_D_INFO, "Platform Info: File name table contains %d entries\n", mPDatFileNameTableLen));

  for (Index=0; Index < mPDatFileNameTableLen; Index++) {

    Status = FindAndCheckPlatformDataFile (
               &mPDatFileNameTable[Index],
               &CurrArea,
               &CurrAreaSize,
               &CurrTypeItem,
               &CurrMrcItem,
               &CurrType
               );

    if (EFI_ERROR (Status)) {
      continue;
    }
    CharOrIndex (Index, &KeyStrForIndex[1], 0, NULL);
    DEBUG (
      (EFI_D_ERROR,
      "Type %a for '%s' [PID %d]\n",
      KeyStrForIndex,
      PlatformTypeString (CurrType),
      (UINTN) CurrType
      ));
  }
}

EFI_STATUS
UserSelectPlatformDataFile (
  OUT PDAT_AREA                           **AreaPtr,
  OUT EFI_GUID                            *PlatformDataFile,
  OUT UINT16                              *TypePtr,
  OUT PDAT_MRC_ITEM                       *MrcConfig
  )
/*++
Routine Description:

  Let user select platform data file to use over debug console.

Arguments:

  AreaPtr          -  Pointer to be pointed to platform data area.
  PlatformDataFile -  GUID to be updated with platform data file name.
  TypePtr          -  Store selected platform type at this address.
  MrcConfig        -  Store mrc config for platform type at this address.

Returns:

  EFI_SUCCESS      -  User selected valid file.
  EFI_NOT_FOUND    -  No file selected.
  AreaPtr          -  Pointing to selected platform data area.
  PlatformDataFile -  Updated with selected file name.
  TypePtr          -  Updated with selected platform type.
  MrcConfig        -  Updated with selected mrc config.

--*/
{
  UINTN                   Selected;
  CHAR8                   Key;
  EFI_STATUS              Status;
  UINTN                   AreaSize;
  PDAT_ITEM               *TypeItem;
  PDAT_ITEM               *MrcItem;

  //
  // Return error if no files to search for.
  //
  if (mPDatFileNameTableLen == 0) {
    return EFI_NOT_FOUND;
  }

  do {
    //
    // Trace list of valid platform data files stored in Firmware volumes.
    //
    TracePlatformDataSelectList ();

    //
    // Ask user to select file to use.
    //
    Key = PlatformDebugPortGetChar8 ();

    //
    // Translate key typed to index into mPDatFileNameTable.
    //
    CharOrIndex (0, NULL, Key, &Selected);
    if (Selected < mPDatFileNameTableLen) {

      //
      // Update output params with selected file.
      //
      Status = FindAndCheckPlatformDataFile (
                 &mPDatFileNameTable[Selected],
                 AreaPtr,
                 &AreaSize,
                 &TypeItem,
                 &MrcItem,
                 TypePtr
                 );
      if (!EFI_ERROR (Status)) {

        //
        // Valid selection return to caller.
        //
        CopyGuid (
          PlatformDataFile,
          &mPDatFileNameTable[Selected]
          );
        CopyMem (
          (VOID *) MrcConfig,
          (VOID *) MrcItem->Data,
          sizeof (PDAT_MRC_ITEM)
          );
        // Use EFI_D_ERROR so user on release builds knows data found.
        DEBUG ((EFI_D_ERROR, "Platform Info: Type = %x\n", (UINTN) *TypePtr));
        DEBUG ((EFI_D_ERROR, "Platform Info: Platform Data Mrc Vars found: length %d version = %d\n",
          (UINTN) MrcItem->Header.Length,
          (UINTN) MrcItem->Header.Version
          ));
        break;
      }
    }
    //
    // Block until user selects valid file.
    //
  } while (TRUE);

  return Status;
}

VOID
GetCpuInfo (
  UINT8     *CpuType,
  UINT8     *CpuStepping
  )
/*++

Routine Description:
   Returns the Model ID of the CPU. 
   Model ID = EAX[7:4] 

Returns:

--*/

{
  UINT32                    RegEax=0;

  AsmCpuid (EFI_CPUID_VERSION_INFO, &RegEax, NULL, NULL, NULL);

  *CpuStepping = (UINT8)(RegEax & 0x0F);
  *CpuType     = (UINT8)((RegEax & 0xF0) >> 4);
}

EFI_STATUS
PdrGetPlatformInfo (
  IN CONST EFI_PEI_SERVICES               **PeiServices,
  OUT EFI_PLATFORM_INFO                   *PlatformInfoHob
  )
/*++
Routine Description:

  Update platform info hob with platform data items.

Arguments:

  PeiServices      -  General purpose services available to every PEIM.
  PlatformInfoHob  -  Hob to update with platform data items.

Returns:

  EFI_SUCCESS  -  Hob updated with platform data items.
  Others       -  All other error conditions encountered result in an ASSERT.

--*/
{
  PDAT_ITEM                         *Item;
  EFI_STATUS                        Status;
  PDAT_AREA                         *Area;
  PDAT_MRC_ITEM                     *MrcItemData;
  QUARK_EDKII_STAGE1_HEADER       *Edk2ImageHeader;

  //
  // Set default values for information derived from platform data area.
  //
  PlatformInfoHob->Type = (EFI_PLATFORM_TYPE) TypeUnknown;
  SetMem (PlatformInfoHob->SysData.IohMac0Address, sizeof(PlatformInfoHob->SysData.IohMac0Address), 0xff);
  SetMem (PlatformInfoHob->SysData.IohMac1Address, sizeof(PlatformInfoHob->SysData.IohMac1Address), 0xff);

  //
  // Return error if platform data CRC error, size error or other unexpected error.
  // For Recovery boot => User selects 'safe embedded' platform data
  //
  Edk2ImageHeader = (QUARK_EDKII_STAGE1_HEADER *) (FixedPcdGet32 (PcdEsramStage1Base) + FixedPcdGet32 (PcdFvSecurityHeaderSize));
  switch ((UINT8)Edk2ImageHeader->ImageIndex & QUARK_STAGE1_IMAGE_TYPE_MASK) {
  case QUARK_STAGE1_RECOVERY_IMAGE_TYPE:
    //
    // Recovery Boot
    //
    Status = UserSelectPlatformDataFile (
      &Area,
      &PlatformInfoHob->BiosPlatformDataFile,
      &PlatformInfoHob->Type,
      &PlatformInfoHob->MemData.MemMrcConfig
      );
    if (EFI_ERROR (Status)) {
      ASSERT_EFI_ERROR (Status);
      return Status;
    }
    break;
  default:
    //
    // Normal Boot
    //
    Status = PDatLibGetSystemAreaPointer (TRUE, &Area);
    if (EFI_ERROR (Status)) {
      if (Status == EFI_NOT_FOUND) {
        DEBUG ((EFI_D_ERROR, "System Platform Data Area Signature not found.\n"));
      } else if (Status == EFI_CRC_ERROR) {
        DEBUG ((EFI_D_ERROR, "System Platform Data Area CRC Error.\n"));
      } else if (Status == EFI_BAD_BUFFER_SIZE) {
        DEBUG ((EFI_D_ERROR, "System Platform Data Area length too large for this platform.\n"));
      } else {
        DEBUG ((EFI_D_ERROR, "System Platform Data Area get failed error = %r.\n", Status));
      }
      ASSERT (FeaturePcdGet (PcdEnableSecureLock) == FALSE);
      Status = UserSelectPlatformDataFile (
        &Area,
        &PlatformInfoHob->BiosPlatformDataFile,
        &PlatformInfoHob->Type,
        &PlatformInfoHob->MemData.MemMrcConfig
        );
      if (EFI_ERROR (Status)) {
        ASSERT_EFI_ERROR (Status);
        return Status;
      }

    } else {
      PlatformInfoHob->Type = (EFI_PLATFORM_TYPE) 0;
      Item = PDatLibFindItem (Area, PDAT_ITEM_ID_PLATFORM_ID, FALSE, &PlatformInfoHob->Type);
      if (Item == NULL) {
        PlatformInfoHob->Type = TypeUnknown;
        DEBUG ((EFI_D_ERROR, "SPI PDR missing does not contain a Platform ID item!!!!\n"));
        ASSERT (FALSE);
      }

      if ((PlatformInfoHob->Type > TypeUnknown)  && (PlatformInfoHob->Type < TypePlatformMax)) {
        //
        // Valid Platform Identified
        //
        DEBUG ((EFI_D_INFO, "Platform Info: Type = %x\n", (UINTN) PlatformInfoHob->Type));
      } else {
        //
        // Reading from SPI PDR Failed or a unknown platform identified
        //
        DEBUG ((EFI_D_WARN, "SPI PDR reports Platform ID as %x. This is unknown ID.\n", PlatformInfoHob->Type));
        PlatformInfoHob->Type = TypeUnknown;
        ASSERT (FALSE);
      }
      Item = PDatLibFindItem (Area, PDAT_ITEM_ID_MRC_VARS, TRUE, NULL);
      if (Item == NULL) {
        DEBUG ((EFI_D_ERROR, "Platform Info: Mrc Vars not found in Platform Data!!!!!\n"));
        ASSERT (FALSE);
      } else {
        Status = CheckMrcParams (PlatformInfoHob->Type, Item);
        ASSERT_EFI_ERROR (Status);
        MrcItemData = (PDAT_MRC_ITEM *) Item->Data;
        CopyMem ((VOID *) &PlatformInfoHob->MemData.MemMrcConfig, (VOID *) MrcItemData, sizeof (PlatformInfoHob->MemData.MemMrcConfig));
        DEBUG ((EFI_D_INFO, "Platform Info: Platform Data Mrc Vars found: length %d version = %d\n",
          (UINTN) Item->Header.Length,
          (UINTN) Item->Header.Version
          ));
      }
    }
    break;
  }

  //
  // Read mac addresses configured in platform data flash area.
  //
  Item = PDatLibFindItem (Area, PDAT_ITEM_ID_MAC0, FALSE, PlatformInfoHob->SysData.IohMac0Address);
  if (Item == NULL) {
    DEBUG ((EFI_D_WARN, "Mac0 address not found in platform data.!!!!\n"));
  } else {
    ASSERT (Item->Header.Length == sizeof(PlatformInfoHob->SysData.IohMac0Address));
  }
  Item = PDatLibFindItem (Area, PDAT_ITEM_ID_MAC1, FALSE, PlatformInfoHob->SysData.IohMac1Address);
  if (Item == NULL) {
    DEBUG ((EFI_D_WARN, "Mac1 address not found in platform data.!!!!\n"));
  } else {
    ASSERT (Item->Header.Length == sizeof(PlatformInfoHob->SysData.IohMac1Address));
  }

  return EFI_SUCCESS;
}

EFI_STATUS
MfhGetPlatformInfo (
  IN CONST EFI_PEI_SERVICES               **PeiServices,
  OUT EFI_PLATFORM_INFO                   *PlatformInfoHob
  )
/*++
Routine Description:

  Update platform info hob with MFH data items.

Arguments:

  PeiServices      -  General purpose services available to every PEIM.
  PlatformInfoHob  -  Hob to update with MFH data items.

Returns:

  EFI_SUCCESS  -  Hob updated with MFH data items.
  Others       -  All other error conditions encountered result in an ASSERT.

--*/
{
  MFH_LIB_FINDCONTEXT               MfhFindContext;
  MFH_FLASH_ITEM                    *FlashItem;

  FlashItem = MfhLibFindFirstWithFilter (
                MFH_FIND_IMAGE_VERSION_FILTER,
                FALSE,
                &MfhFindContext
                );
  ASSERT (FlashItem != NULL);

  PlatformInfoHob->FirmwareVersion = FlashItem->TypeSpecific.ImageVersion;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
PlatformInfoInit (
  IN       EFI_PEI_FILE_HANDLE  FileHandle,
  IN CONST EFI_PEI_SERVICES     **PeiServices
  )
/*++
Routine Description:

  Platform Type detection. Because the PEI globle variable 
  is in the flash, it could not change directly.So use 
  2 PPIs to distinguish the platform type.

Arguments:

  FfsHeader    -  Pointer to Firmware File System file header.
  PeiServices  -  General purpose services available to every PEIM.

Returns:

  EFI_SUCCESS  -  Memory initialization completed successfully.
  Others       -  All other error conditions encountered result in an ASSERT.

--*/
{
  EFI_STATUS              Status;
  EFI_PEI_PCI_CFG2_PPI    *PciCfgPpi;
  UINT8                   CpuType;
  UINT8                   CpuStepping;
  UINT16                  Data16;
  UINT8                   Data8;
  EFI_PLATFORM_INFO       PlatformInfoHob;

  PciCfgPpi = (**PeiServices).PciCfg;
  ASSERT (PciCfgPpi != NULL);

  (*PeiServices)->SetMem (
                    &PlatformInfoHob,
                    sizeof (PlatformInfoHob),
                    0
                    );

  //
  // Update platform info hob with platform data items.
  //
  Status = PdrGetPlatformInfo(PeiServices, &PlatformInfoHob);
  ASSERT_EFI_ERROR (Status);

  //
  // Update platform info hob with MFH data items if not recovery boot.
  //
  if (!PlatformIsBootWithRecoveryStage1 ()) {
    Status = MfhGetPlatformInfo(PeiServices, &PlatformInfoHob);
    ASSERT_EFI_ERROR (Status);
  }

  //
  // Update IIO Type
  //
  PlatformInfoHob.IioRevision = 0;

  //
  // Update QNC Type
  //
  //
  // Device ID
  //
  PciCfgPpi->Read (
              PeiServices,
              PciCfgPpi,
              EfiPeiPciCfgWidthUint16,
              PEI_PCI_CFG_ADDRESS (MC_BUS, MC_DEV, MC_FUN, PCI_DEVICE_ID_OFFSET),
              &Data16
              );
  PlatformInfoHob.QncSku= Data16;

  PciCfgPpi->Read (
              PeiServices,
              PciCfgPpi,
              EfiPeiPciCfgWidthUint8,
              PEI_PCI_CFG_ADDRESS (MC_BUS, MC_DEV, MC_FUN, PCI_REVISION_ID_OFFSET),
              &Data8
              );
  PlatformInfoHob.QncRevision = Data8;

  PlatformInfoHob.SysData.SysSioExist = FALSE;

  GetCpuInfo (&CpuType, &CpuStepping);
  PlatformInfoHob.CpuType     = CpuType;
  PlatformInfoHob.CpuStepping = CpuStepping;

  //
  // Build HOB for setup memory information
  //

  BuildGuidDataHob (
      &gEfiPlatformInfoGuid,
      &(PlatformInfoHob),
      sizeof (EFI_PLATFORM_INFO)
      );

  Status = (**PeiServices).InstallPpi (PeiServices, &mPlatformInfoPpi);
  ASSERT_EFI_ERROR (Status);

  return Status;
}
