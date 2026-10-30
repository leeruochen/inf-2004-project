/*
 *  ipc.c
 *  ipc_init(): creates every IPC object of design.md 3.3 at its design depth,
 *  proves each message buffer's depth, and checks the kernel version.
 *  Owner: platform (Batch B, stage B2).  Kernel side: kernel headers only.
 *
 *  Runs on the 1 KB init stack (usermain step 1): locals stay small (one
 *  16-byte dummy message, one T_RMBF), and logging uses dbg_puts /
 *  dbg_put_err only, never dbg_printf.
 */

#include "src/common/ipc.h"
#include "src/common/mbf_size.h"
#include "src/debug/dbg.h"

ID ipc_mbf_nav_in;
ID ipc_mbf_motion_cmd;
ID ipc_mbf_obstacle;
ID ipc_mbf_telemetry;
ID ipc_sem_echo;
ID ipc_evf_safety;
ID ipc_evf_nav;
ID ipc_mtx_motion;

/* Largest message any buffer carries; the self-check sends this dummy. */
#define IPC_DUMMY_WORDS     (4U)
_Static_assert(sizeof(nav_input_t) <= (IPC_DUMMY_WORDS * 4U), "dummy too small");
_Static_assert(sizeof(motion_cmd_t) <= (IPC_DUMMY_WORDS * 4U), "dummy too small");
_Static_assert(sizeof(obstacle_profile_t) <= (IPC_DUMMY_WORDS * 4U), "dummy too small");
_Static_assert(sizeof(tel_evt_t) <= (IPC_DUMMY_WORDS * 4U), "dummy too small");

/* tk_ref_ver() reports the micro T-Kernel magic nibble with the version:
 * spver = 0x6000 | (major << 8) | minor (sys/knldef.h:230), i.e. 0x6300. */
#define IPC_EXPECTED_SPVER  ((UH)(((UW)TK_SPECVER_MAGIC << 12) | (UW)TK_SPECVER))

typedef struct
{
    ID         *p_id;
    uint32_t    depth;
    uint32_t    msg_bytes;
    char const *p_create;       /* log text if creation fails */
    char const *p_check;        /* log text if the depth self-check fails */
} ipc_mbf_desc_t;

static ipc_mbf_desc_t const k_mbfs[] =
{
    { &ipc_mbf_nav_in, MBF_NAV_IN_DEPTH, sizeof(nav_input_t),
      "ipc: create MBF_NAV_IN", "ipc: depth check MBF_NAV_IN" },
    { &ipc_mbf_motion_cmd, MBF_MOTION_CMD_DEPTH, sizeof(motion_cmd_t),
      "ipc: create MBF_MOTION_CMD", "ipc: depth check MBF_MOTION_CMD" },
    { &ipc_mbf_obstacle, MBF_OBSTACLE_DEPTH, sizeof(obstacle_profile_t),
      "ipc: create MBF_OBSTACLE", "ipc: depth check MBF_OBSTACLE" },
    { &ipc_mbf_telemetry, MBF_TELEMETRY_DEPTH, sizeof(tel_evt_t),
      "ipc: create MBF_TELEMETRY", "ipc: depth check MBF_TELEMETRY" },
};

/* Keep the first error; log every one. */
static void note_error(ER *p_first, char const *p_what, ER er)
{
    dbg_put_err(p_what, (int32_t)er);
    if (*p_first == E_OK)
    {
        *p_first = er;
    }
}

/*
 * Depth self-check on an empty buffer (integration report section 9):
 * free size == buffer size; N sends with TMO_POL succeed; send N+1 returns
 * E_TMOUT; N receives return the message size; one more returns E_TMOUT;
 * the buffer is empty again.  Returns E_OK, a kernel error, or E_SYS for a
 * wrong size or count.
 */
static ER mbf_self_check(ipc_mbf_desc_t const *p_desc)
{
    uint32_t       dummy[IPC_DUMMY_WORDS] = { 0U };
    T_RMBF         ref;
    ER             er;
    INT            n;
    uint32_t       i;
    ID const       id = *p_desc->p_id;
    uint32_t const bufsz = p_desc->depth * MBF_SLOT_BYTES(p_desc->msg_bytes);

    er = tk_ref_mbf(id, &ref);
    if (er != E_OK)
    {
        return er;
    }
    if (((uint32_t)ref.frbufsz != bufsz) || ((uint32_t)ref.maxmsz != p_desc->msg_bytes))
    {
        return E_SYS;
    }

    for (i = 0U; i < p_desc->depth; i++)
    {
        er = tk_snd_mbf(id, dummy, (INT)p_desc->msg_bytes, TMO_POL);
        if (er != E_OK)
        {
            return er;
        }
    }
    er = tk_snd_mbf(id, dummy, (INT)p_desc->msg_bytes, TMO_POL);
    if (er != E_TMOUT)
    {
        return (er < E_OK) ? er : E_SYS;
    }

    for (i = 0U; i < p_desc->depth; i++)
    {
        n = tk_rcv_mbf(id, dummy, TMO_POL);
        if (n != (INT)p_desc->msg_bytes)
        {
            return (n < E_OK) ? (ER)n : E_SYS;
        }
    }
    n = tk_rcv_mbf(id, dummy, TMO_POL);
    if (n != E_TMOUT)
    {
        return (n < E_OK) ? (ER)n : E_SYS;
    }

    er = tk_ref_mbf(id, &ref);
    if (er != E_OK)
    {
        return er;
    }
    return ((uint32_t)ref.frbufsz == bufsz) ? E_OK : E_SYS;
}

static void create_mbfs(ER *p_first)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)(sizeof(k_mbfs) / sizeof(k_mbfs[0])); i++)
    {
        ipc_mbf_desc_t const *p_desc = &k_mbfs[i];
        T_CMBF const          cmbf =
        {
            .exinf  = NULL,
            .mbfatr = TA_TFIFO,
            .bufsz  = (SZ)(p_desc->depth * MBF_SLOT_BYTES(p_desc->msg_bytes)),
            .maxmsz = (INT)p_desc->msg_bytes,
            .bufptr = NULL,
        };
        ID const id = tk_cre_mbf(&cmbf);

        if (id < E_OK)
        {
            note_error(p_first, p_desc->p_create, id);
        }
        else
        {
            ER er;

            *p_desc->p_id = id;
            er = mbf_self_check(p_desc);
            if (er != E_OK)
            {
                note_error(p_first, p_desc->p_check, er);
            }
        }
    }
}

static void create_others(ER *p_first)
{
    /* IF-05 SEM_ECHO: a doorbell from the echo ISR.  Max count 1: a second
     * signal before the task runs adds nothing.
     * TODO(owner-to-confirm, buddy 5): max count. */
    T_CSEM const csem = { .exinf = NULL, .sematr = TA_TFIFO, .isemcnt = 0, .maxsem = 1 };
    /* IF-06 EVF_SAFETY: only Safety waits (TA_WSGL, ARCHITECTURE.md D9). */
    T_CFLG const cflg_safety = { .exinf = NULL, .flgatr = TA_TFIFO | TA_WSGL, .iflgptn = 0U };
    /* IF-07 EVF_NAV: readers poll; TA_WMUL so a TMO_POL wait never fails with E_OBJ. */
    T_CFLG const cflg_nav = { .exinf = NULL, .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0U };
    /* IF-08 MTX_MOTION: priority inheritance (design.md 3.3). */
    T_CMTX const cmtx = { .exinf = NULL, .mtxatr = TA_INHERIT, .ceilpri = 0 };
    ID id;

    id = tk_cre_sem(&csem);
    if (id < E_OK)
    {
        note_error(p_first, "ipc: create SEM_ECHO", id);
    }
    else
    {
        ipc_sem_echo = id;
    }

    id = tk_cre_flg(&cflg_safety);
    if (id < E_OK)
    {
        note_error(p_first, "ipc: create EVF_SAFETY", id);
    }
    else
    {
        ipc_evf_safety = id;
    }

    id = tk_cre_flg(&cflg_nav);
    if (id < E_OK)
    {
        note_error(p_first, "ipc: create EVF_NAV", id);
    }
    else
    {
        ipc_evf_nav = id;
    }

    id = tk_cre_mtx(&cmtx);
    if (id < E_OK)
    {
        note_error(p_first, "ipc: create MTX_MOTION", id);
    }
    else
    {
        ipc_mtx_motion = id;
    }
}

static void check_kernel_version(ER *p_first)
{
    T_RVER ver;
    ER     er = tk_ref_ver(&ver);

    if (er != E_OK)
    {
        note_error(p_first, "ipc: tk_ref_ver", er);
    }
    else if (ver.spver != IPC_EXPECTED_SPVER)
    {
        note_error(p_first, "ipc: kernel spver is not 0x6300, got", (ER)ver.spver);
    }
    else
    {
        dbg_puts("[IPC] kernel spec version 3.00 OK");
    }
}

ER ipc_init(void)
{
    ER first = E_OK;

    create_mbfs(&first);
    create_others(&first);
    check_kernel_version(&first);

    if (first == E_OK)
    {
        dbg_puts("[IPC] 8 objects created, MBF depth self-check OK");
    }
    return first;
}
