/**
 * @file
 * @copyright  Copyright 2016 GNSS Sensor Ltd. All right reserved.
 * @author     Sergey Khabarov - sergeykhbr@gmail.com
 * @brief      elf-file loader class declaration.
 */

#ifndef __DEBUGGER_ELF_LOADER_H__
#define __DEBUGGER_ELF_LOADER_H__

#include "iclass.h"
#include "iservice.h"
#include "itap.h"
#include "ielfloader.h"
#include "elf_types.h"
#include "bfd.h"

namespace debugger {

class ElfLoaderService : public IService,
                         public IElfLoader {
public:
    explicit ElfLoaderService(const char *name);
    virtual ~ElfLoaderService();

    /** IService interface */
    virtual void postinitService();

    /** IElfLoader interface */
    virtual int loadFile(const char *filename);

private:
    int loadSections(bfd *tmp_bfd);
    uint64_t loadMemory(uint64_t addr, uint8_t *buf, uint64_t bufsz);
    uint64_t initMemory(uint64_t addr, uint64_t bufsz);

private:
    ITap *itap_;
    AttributeType tap_;
    AttributeType verify_ena_;  // read and compare memory after write

    uint8_t *image_;
    ElfHeaderType *header_;
    SectionHeaderType *sh_tbl_;
    char *sectionNames_;
    char *symbolNames_;
};

DECLARE_CLASS(ElfLoaderService)

}  // namespace debugger

#endif  // __DEBUGGER_ELF_LOADER_H__
