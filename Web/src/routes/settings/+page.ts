export const prerender = false;
export const csr = true;

export const load = async ({ fetch }) => {
    const res = await fetch('/api/settings');
    if (res.status === 200) {
        const data = await res.json();
        return { settings: data };
    }
    return { settings: undefined };
};
